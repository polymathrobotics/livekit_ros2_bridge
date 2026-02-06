# Copyright (c) 2025-present Polymath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Callable

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from livekit_ros2_bridge.core.access import (
    AccessOperation,
    AccessPolicy,
    AccessResource,
    AccessDecision,
)
from livekit_ros2_bridge.core.request_context import RequestContext
from livekit_ros2_bridge.core.names import normalize_ros_topic
from livekit_ros2_bridge.core.protocol import (
    LivekitRosSubscriptionInfo,
    RosSubscriptionReliability,
    LivekitRpcSubscribeRequest,
    LivekitRpcSubscriptionStatus,
    LivekitRpcUnsubscribeRequest,
)
from livekit_ros2_bridge.core.room_publisher import RoomPublisher
from livekit_ros2_bridge.core.telemetry import (
    NullTelemetry,
    AccessDenyTelemetryEvent,
    Telemetry,
)
from livekit_ros2_bridge.ros2.topic_subscription import RosTopicSubscription

logger = logging.getLogger(__name__)


def _default_subscription_qos() -> QoSProfile:
    profile = QoSProfile(depth=10)
    profile.reliability = QoSReliabilityPolicy.RELIABLE
    profile.durability = QoSDurabilityPolicy.VOLATILE
    return profile


@dataclass(frozen=True)
class SubscriberConfig:
    max_total_subscriptions: int = 128
    max_subscriptions_per_participant: int = 32


class RosSubscriptionRegistry:
    """Registry of active ROS subscriptions keyed by normalized topic.

    Public role: track subscription membership and lifecycle for subscribe/unsubscribe RPCs.
    Per-topic streaming lives in RosTopicSubscription; keep business logic outside this registry.

    Threading: all methods must be called from the ROS executor thread.
    """

    def __init__(
        self,
        node: Node,
        room_publisher: RoomPublisher,
        config: SubscriberConfig,
        *,
        access_policy: AccessPolicy,
        telemetry: Telemetry | None = None,
        callback_group: CallbackGroup | None = None,
    ) -> None:
        self._node = node
        self._room_publisher = room_publisher
        self._access_policy = access_policy
        self._telemetry = telemetry or NullTelemetry()
        self._callback_group = callback_group

        self._subscriptions_by_topic: dict[str, RosTopicSubscription] = {}
        self._max_total_subscriptions = max(int(config.max_total_subscriptions), 0)
        self._max_subscriptions_per_participant = max(
            int(config.max_subscriptions_per_participant), 0
        )
        self._message_id = 0

    def subscribe_request(
        self,
        ctx: RequestContext,
        rpc_request: LivekitRpcSubscribeRequest,
    ) -> tuple[LivekitRosSubscriptionInfo, LivekitRpcSubscriptionStatus]:
        topic = self._normalize_topic_or_raise(rpc_request.topic)
        requester_id = self._requester_id_or_raise(ctx)
        preferred_interval_ms = int(rpc_request.preferred_interval_ms)

        subscription = self._subscriptions_by_topic.get(topic)
        if subscription is not None:
            self._authorize_subscribe_or_raise(ctx, topic, subscription.info.type)
            if requester_id not in subscription.requesters:
                self._enforce_limits(
                    requester_id,
                    creating_new_subscription=False,
                )
            subscription.add_requester(requester_id, preferred_interval_ms)
            status = self._status_for(subscription)
            logger.info(
                "Updated LiveKit ROS subscription: ros_topic=%s requester_id=%s preferred_interval_ms=%s "
                "applied_interval_ms=%s requester_count=%s",
                topic,
                requester_id,
                preferred_interval_ms,
                status.applied_interval_ms,
                status.requester_count,
            )
            return subscription.info, status

        self._enforce_limits(requester_id, creating_new_subscription=True)
        subscription = self._create_subscription(
            ctx=ctx,
            ros_topic=topic,
            requester_id=requester_id,
            preferred_interval_ms=preferred_interval_ms,
        )
        self._subscriptions_by_topic[topic] = subscription
        status = self._status_for(subscription)
        logger.info(
            "Created LiveKit ROS subscription: ros_topic=%s requester_id=%s preferred_interval_ms=%s "
            "applied_interval_ms=%s requester_count=%s",
            topic,
            requester_id,
            preferred_interval_ms,
            status.applied_interval_ms,
            status.requester_count,
        )
        return subscription.info, status

    def unsubscribe_request(
        self,
        ctx: RequestContext,
        rpc_request: LivekitRpcUnsubscribeRequest,
    ) -> tuple[LivekitRosSubscriptionInfo, LivekitRpcSubscriptionStatus]:
        topic = self._normalize_topic_or_raise(rpc_request.topic)
        requester_id = self._requester_id_or_raise(ctx)

        subscription = self._subscriptions_by_topic.get(topic)
        if subscription is None:
            raise KeyError(f"No active subscription for topic '{topic}'.")
        if requester_id not in subscription.requesters:
            raise KeyError(f"Requester '{requester_id}' not subscribed to '{topic}'.")

        subscription.remove_requester(requester_id)
        remaining_count = subscription.requester_count()
        destroyed = False
        if remaining_count == 0:
            self._subscriptions_by_topic.pop(topic, None)
            self._destroy_subscription(subscription)
            destroyed = True

        status = LivekitRpcSubscriptionStatus(
            applied_interval_ms=subscription.applied_interval_ms
            if remaining_count
            else 0,
            requester_count=remaining_count,
        )
        logger.info(
            "LiveKit ROS unsubscribe: ros_topic=%s requester_id=%s requester_count=%s "
            "applied_interval_ms=%s destroyed=%s",
            topic,
            requester_id,
            remaining_count,
            status.applied_interval_ms,
            destroyed,
        )
        return subscription.info, status

    def remove_participant(self, requester_id: str) -> None:
        if not requester_id:
            return
        logger.info(
            "Removing LiveKit ROS subscriptions for requester_id=%s",
            requester_id,
        )
        self._drop_requesters(
            lambda active_requester_id: active_requester_id == requester_id
        )

    def clear_livekit_requesters(self) -> None:
        logger.info("Clearing LiveKit ROS subscriptions for all requesters.")
        self._drop_requesters(lambda _requester_id: True)

    def shutdown(self) -> None:
        subscriptions = list(self._subscriptions_by_topic.values())
        self._subscriptions_by_topic.clear()
        for subscription in subscriptions:
            try:
                self._destroy_subscription(subscription)
            except Exception as exc:
                logger.error(
                    "Failed to destroy ROS subscription for topic %s: %s",
                    subscription.info.topic,
                    exc,
                    exc_info=True,
                )

    def _normalize_topic_or_raise(self, topic: str) -> str:
        normalized = normalize_ros_topic(topic)
        if normalized:
            return normalized
        raise ValueError(f"Invalid ROS topic: {topic!r}")

    def _requester_id_or_raise(self, ctx: RequestContext) -> str:
        requester_id = str(ctx.requester_id or "").strip()
        if requester_id:
            return requester_id
        raise ValueError("requester_id is required")

    def _status_for(
        self,
        subscription: RosTopicSubscription,
    ) -> LivekitRpcSubscriptionStatus:
        return LivekitRpcSubscriptionStatus(
            applied_interval_ms=subscription.applied_interval_ms,
            requester_count=subscription.requester_count(),
        )

    def _create_subscription(
        self,
        *,
        ctx: RequestContext,
        ros_topic: str,
        requester_id: str,
        preferred_interval_ms: int,
    ) -> RosTopicSubscription:
        ros_type = self._select_allowed_type_or_raise(ctx, ros_topic)

        try:
            msg_type = get_message(ros_type)
        except Exception as exc:
            raise ValueError(f"Unknown ROS message type: {ros_type}") from exc

        info = LivekitRosSubscriptionInfo(
            topic=ros_topic,
            type=ros_type,
            reliability=RosSubscriptionReliability.RELIABLE,
            depth=10,
        )
        return RosTopicSubscription(
            node=self._node,
            room_publisher=self._room_publisher,
            info=info,
            qos_profile=_default_subscription_qos(),
            msg_type=msg_type,
            callback_group=self._callback_group,
            message_id_provider=self._next_message_id,
            requesters={requester_id: preferred_interval_ms},
            telemetry=self._telemetry,
        )

    def _resolve_topic_types(self, ros_topic: str) -> list[str]:
        try:
            topics = self._node.get_topic_names_and_types()
        except Exception as exc:
            logger.error("Failed to resolve ROS topic types: %s", exc, exc_info=True)
            return []

        for name, types in topics:
            if name == ros_topic:
                return list(types)
        return []

    def _select_allowed_type_or_raise(self, ctx: RequestContext, ros_topic: str) -> str:
        topic_types = self._resolve_topic_types(ros_topic)
        if not topic_types:
            raise LookupError(f"No ROS message types found for topic '{ros_topic}'.")

        allowed_types: list[str] = []
        first_denied: AccessDecision | None = None
        for ros_type in topic_types:
            decision = self._access_policy.authorize(
                ctx,
                AccessOperation.SUBSCRIBE,
                AccessResource(name=ros_topic, ros_type=ros_type),
            )
            if decision.ok:
                allowed_types.append(ros_type)
            elif first_denied is None:
                first_denied = decision

        if not allowed_types:
            denied = first_denied or AccessDecision(ok=False, reason="subscribe_denied")
            self._record_access_denial(
                ctx,
                AccessOperation.SUBSCRIBE,
                ros_topic,
                denied,
            )
            if denied.reason == "subscribe_type_denied":
                raise PermissionError(
                    f"ROS message type for topic '{ros_topic}' is not permitted."
                )
            raise PermissionError(f"ROS topic '{ros_topic}' not permitted.")

        if len(allowed_types) > 1:
            raise ValueError(
                "Unsupported multi-type ROS topic for subscription request "
                f"'{ros_topic}': {', '.join(topic_types)}. "
                "The subscribe RPC does not include a message type, so the request cannot disambiguate."
            )
        return allowed_types[0]

    def _authorize_subscribe_or_raise(
        self,
        ctx: RequestContext,
        ros_topic: str,
        ros_type: str,
    ) -> None:
        decision = self._access_policy.authorize(
            ctx,
            AccessOperation.SUBSCRIBE,
            AccessResource(name=ros_topic, ros_type=ros_type),
        )
        if decision.ok:
            return
        self._record_access_denial(ctx, AccessOperation.SUBSCRIBE, ros_topic, decision)
        if decision.reason == "subscribe_type_denied":
            raise PermissionError(f"ROS message type '{ros_type}' is not permitted.")
        raise PermissionError(f"ROS topic '{ros_topic}' not permitted.")

    def _record_access_denial(
        self,
        ctx: RequestContext,
        op: AccessOperation,
        resource: str,
        decision: AccessDecision,
    ) -> None:
        try:
            self._telemetry.emit(
                ctx,
                AccessDenyTelemetryEvent(
                    op=op,
                    resource=normalize_ros_topic(resource) or resource,
                    reason=decision.reason,
                ),
            )
        except Exception:
            logger.debug("Telemetry.emit failed for access denial", exc_info=True)

    def _drop_requesters(self, should_remove: Callable[[str], bool]) -> None:
        to_destroy: list[RosTopicSubscription] = []
        for topic, subscription in list(self._subscriptions_by_topic.items()):
            removed = [
                requester_id
                for requester_id in subscription.requesters
                if should_remove(requester_id)
            ]
            if not removed:
                continue

            subscription.remove_requesters(removed)
            if subscription.requesters:
                logger.info(
                    "LiveKit ROS subscription requesters removed: ros_topic=%s removed=%s "
                    "applied_interval_ms=%s requester_count=%s",
                    topic,
                    removed,
                    subscription.applied_interval_ms,
                    subscription.requester_count(),
                )
                continue

            self._subscriptions_by_topic.pop(topic, None)
            to_destroy.append(subscription)
            logger.info(
                "LiveKit ROS subscription requesters removed: ros_topic=%s removed=%s requester_count=0",
                topic,
                removed,
            )

        for subscription in to_destroy:
            self._destroy_subscription(subscription)

    def _destroy_subscription(self, subscription: RosTopicSubscription) -> None:
        if subscription.destroy():
            return
        logger.warning(
            "Failed to destroy ROS subscription for topic %s",
            subscription.info.topic,
        )

    def _enforce_limits(
        self,
        requester_id: str,
        *,
        creating_new_subscription: bool,
    ) -> None:
        if (
            creating_new_subscription
            and self._max_total_subscriptions
            and len(self._subscriptions_by_topic) >= self._max_total_subscriptions
        ):
            raise PermissionError("Maximum active ROS subscriptions reached.")

        if not self._max_subscriptions_per_participant:
            return

        count = sum(
            1
            for subscription in self._subscriptions_by_topic.values()
            if requester_id in subscription.requesters
        )
        if count >= self._max_subscriptions_per_participant:
            raise PermissionError("Requester subscription limit reached.")

    def _next_message_id(self) -> str:
        self._message_id += 1
        return f"msg-{self._message_id}"


__all__ = ["RosSubscriptionRegistry", "SubscriberConfig"]
