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

import json
import logging
import time
from typing import Any, Callable, Iterable

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription
from rosidl_runtime_py.convert import message_to_ordereddict

from livekit_ros2_bridge.core.protocol import (
    DATA_CONTENT_TYPE,
    DATA_TOPIC,
    PROTOCOL_VERSION,
    LivekitRosMessageBody,
    LivekitRosMessageEnvelope,
    LivekitRosSubscriptionInfo,
)
from livekit_ros2_bridge.core.serialization import sanitize_payload
from livekit_ros2_bridge.core.telemetry import (
    EgressMessageTelemetryEvent,
    NullTelemetry,
    Telemetry,
)
from livekit_ros2_bridge.core.room_publisher import RoomPublisher

logger = logging.getLogger(__name__)


class RosTopicSubscription:
    """One ROS subscription that forwards messages to the LiveKit data channel."""

    def __init__(
        self,
        *,
        node: Node,
        room_publisher: RoomPublisher,
        info: LivekitRosSubscriptionInfo,
        qos_profile: QoSProfile,
        msg_type: type,
        callback_group: CallbackGroup | None,
        message_id_provider: Callable[[], str],
        requesters: dict[str, int] | None = None,
        telemetry: Telemetry | None = None,
    ) -> None:
        self._node = node
        self._room_publisher = room_publisher
        self._telemetry = telemetry or NullTelemetry()
        self._message_id_provider = message_id_provider

        self.info = info
        self.subscription: Subscription = node.create_subscription(
            msg_type,
            info.topic,
            self._ros_callback,
            qos_profile,
            callback_group=callback_group,
        )
        self.requesters = dict(requesters or {})
        self.applied_interval_ms = 0
        self.last_sent_monotonic_s = 0.0
        self._update_applied_interval()

    def add_requester(self, requester_id: str, preferred_interval_ms: int) -> None:
        self.requesters[requester_id] = preferred_interval_ms
        self._update_applied_interval()

    def remove_requester(self, requester_id: str) -> None:
        self.requesters.pop(requester_id, None)
        self._update_applied_interval()

    def remove_requesters(self, requester_ids: Iterable[str]) -> list[str]:
        removed: list[str] = []
        for requester_id in requester_ids:
            if self.requesters.pop(requester_id, None) is not None:
                removed.append(requester_id)
        if removed:
            self._update_applied_interval()
        return removed

    def requester_count(self) -> int:
        return len(self.requesters)

    def destroy(self) -> bool:
        return self._node.destroy_subscription(self.subscription)

    def _update_applied_interval(self) -> None:
        self.applied_interval_ms = min(self.requesters.values(), default=0)

    def _ros_callback(self, msg: Any) -> None:
        if not self.requesters:
            return

        if not self._room_publisher.can_publish_data():
            return

        if self._should_skip_due_to_interval():
            return

        payload = self._serialize_message(msg)
        if payload is None:
            return

        destination_identities = list(self.requesters.keys())
        message_id = self._message_id_provider()
        envelope = LivekitRosMessageEnvelope(
            v=PROTOCOL_VERSION,
            type="ros.message",
            id=message_id,
            body=LivekitRosMessageBody(
                topic=self.info.topic,
                type=self.info.type,
                msg=payload,
                content_type=DATA_CONTENT_TYPE,
            ),
        ).dict(exclude_none=False)
        try:
            encoded = json.dumps(envelope, allow_nan=False).encode("utf-8")
            self._telemetry.emit(
                None,
                EgressMessageTelemetryEvent(
                    topic=self.info.topic,
                    ros_type=self.info.type,
                    bytes_len=len(encoded),
                    destinations=len(destination_identities),
                ),
            )
        except Exception:
            logger.debug("Telemetry.emit failed for egress_message", exc_info=True)

        self._room_publisher.publish_data(
            DATA_TOPIC,
            envelope,
            reliable=True,
            destination_identities=destination_identities,
        )

    def _should_skip_due_to_interval(self) -> bool:
        interval_ms = self.applied_interval_ms
        if not interval_ms:
            return False
        now = time.monotonic()
        interval_s = interval_ms / 1000.0
        if now - self.last_sent_monotonic_s < interval_s:
            return True
        self.last_sent_monotonic_s = now
        return False

    def _serialize_message(self, msg: Any) -> Any | None:
        try:
            payload = message_to_ordereddict(msg)
            return sanitize_payload(payload)
        except Exception as exc:
            logger.error(
                "Failed to serialize ROS message for subscription %s: %s",
                self.info.topic,
                exc,
                exc_info=True,
            )
            return None


__all__ = ["RosTopicSubscription"]
