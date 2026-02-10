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
import logging
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any

from pydantic import ValidationError
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message

from livekit_ros2_bridge.core.access import (
    AccessOperation,
    AccessPolicy,
    AccessResource,
    AccessDecision,
)
from livekit_ros2_bridge.core.request_context import RequestContext, RequestSource
from livekit_ros2_bridge.core.names import normalize_ros_topic
from livekit_ros2_bridge.core.protocol import LivekitRosPublishPayload
from livekit_ros2_bridge.core.telemetry import (
    NullTelemetry,
    AccessDenyTelemetryEvent,
    Telemetry,
)

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class PublisherConfig:
    max_topics: int = 50


class RosPublisher:
    """Translate LiveKit publish payloads into ROS publications."""

    def __init__(
        self,
        ros_publisher_registry: Any,
        config: PublisherConfig,
        *,
        access_policy: AccessPolicy,
        telemetry: Telemetry | None = None,
    ) -> None:
        self._ros_publisher_registry = ros_publisher_registry
        self._access_policy = access_policy
        self._telemetry = telemetry or NullTelemetry()
        self._max_topics = max(int(config.max_topics), 0)
        self._seen_topics: OrderedDict[str, None] = OrderedDict()
        self._expected_types: dict[str, str] = {}

    def handle_publish_payload(
        self,
        payload: Any,
        *,
        ctx: RequestContext | None = None,
    ) -> AccessDecision:
        ctx = ctx or RequestContext(
            requester_id=None, source=RequestSource.LIVEKIT_DATA
        )
        parsed = self._parse_payload(payload)
        if parsed is None:
            return AccessDecision(ok=False, reason="invalid_payload")

        topic = normalize_ros_topic(parsed.topic)
        if not topic:
            logger.warning(
                "Ignoring publish payload with invalid topic: %s", parsed.topic
            )
            return AccessDecision(ok=False, reason="invalid_topic")

        decision = self._access_policy.authorize(
            ctx,
            AccessOperation.PUBLISH,
            AccessResource(name=topic, ros_type=parsed.type),
        )
        if not decision.ok:
            logger.warning(
                "Ignoring publish payload for denied topic: %s requester_id=%s reason=%s",
                topic,
                ctx.requester_id or "",
                decision.reason,
            )
            try:
                self._telemetry.emit(
                    ctx,
                    AccessDenyTelemetryEvent(
                        op=AccessOperation.PUBLISH,
                        resource=topic,
                        reason=decision.reason,
                    ),
                )
            except Exception:
                logger.debug("Telemetry.emit failed for publish deny", exc_info=True)
            return decision

        type_error = self._validate_topic_type(topic, parsed.type)
        if type_error is not None:
            return type_error

        try:
            msg_cls = get_message(parsed.type)
        except Exception:
            logger.error(
                "Unknown ROS message type for publish payload: %s",
                parsed.type,
                exc_info=True,
            )
            return AccessDecision(ok=False, reason="unknown_ros_type")

        try:
            msg_instance = msg_cls()
            set_message_fields(msg_instance, parsed.msg)
        except Exception as exc:
            logger.error(
                "Error setting ROS message fields for topic=%s type=%s: %s",
                topic,
                parsed.type,
                exc,
                exc_info=True,
            )
            return AccessDecision(ok=False, reason="invalid_ros_message")

        try:
            self._ros_publisher_registry.publish_message(topic, msg_instance)
        except Exception as exc:
            logger.error(
                "Error publishing ROS message to topic=%s: %s",
                topic,
                exc,
                exc_info=True,
            )
            return AccessDecision(ok=False, reason="ros_publish_error")

        self._remember_topic(topic)
        return AccessDecision(ok=True)

    def _parse_payload(self, payload: Any) -> LivekitRosPublishPayload | None:
        if not isinstance(payload, dict):
            logger.warning(
                "Ignoring non-dict publish payload of type %s",
                type(payload),
            )
            return None
        try:
            return LivekitRosPublishPayload.parse_obj(payload)
        except ValidationError:
            logger.warning("Ignoring publish payload without valid topic/type/msg.")
            return None

    def _validate_topic_type(
        self,
        topic: str,
        requested_type: str,
    ) -> AccessDecision | None:
        expected_type = self._expected_types.get(topic)
        if expected_type is None:
            graph_types = self._ros_publisher_registry.resolve_topic_types(topic)
            if not graph_types:
                logger.warning(
                    "Rejecting publish payload for topic=%s: no ROS graph type is available",
                    topic,
                )
                return AccessDecision(ok=False, reason="topic_type_unknown")
            if len(graph_types) > 1:
                logger.warning(
                    "Rejecting publish payload for topic=%s: multiple ROS graph types: %s",
                    topic,
                    ", ".join(graph_types),
                )
                return AccessDecision(ok=False, reason="topic_type_ambiguous")
            expected_type = graph_types[0]
            self._expected_types[topic] = expected_type

        if expected_type == requested_type:
            return None

        logger.warning(
            "Rejecting publish payload for topic=%s: type mismatch expected=%s got=%s",
            topic,
            expected_type,
            requested_type,
        )
        return AccessDecision(ok=False, reason="topic_type_mismatch")

    def _remember_topic(self, ros_topic: str) -> None:
        if ros_topic in self._seen_topics:
            self._seen_topics.move_to_end(ros_topic)
        else:
            self._seen_topics[ros_topic] = None
        if not self._max_topics:
            return

        while len(self._seen_topics) > self._max_topics:
            evicted_topic, _ = self._seen_topics.popitem(last=False)
            self._expected_types.pop(evicted_topic, None)
            removed = self._ros_publisher_registry.unregister_publisher(evicted_topic)
            logger.info(
                "Publisher topic cap reached; evicted topic=%s to allow topic=%s (publisher_removed=%s)",
                evicted_topic,
                ros_topic,
                removed,
            )


__all__ = ["RosPublisher", "PublisherConfig"]
