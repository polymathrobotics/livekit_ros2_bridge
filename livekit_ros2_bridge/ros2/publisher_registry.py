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
from typing import Any

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from livekit_ros2_bridge.core.names import normalize_ros_topic

logger = logging.getLogger(__name__)


def _make_default_qos_profile() -> QoSProfile:
    profile = QoSProfile(depth=10)
    profile.reliability = QoSReliabilityPolicy.RELIABLE
    profile.durability = QoSDurabilityPolicy.VOLATILE
    return profile


class RosPublisherRegistry:
    """Registry of ROS publishers keyed by normalized topic.

    Public role: own publisher lookup/create/reuse/destroy for publish paths.
    Keep business logic outside this class so it stays an infrastructure registry.
    """

    def __init__(
        self,
        node: Node,
        *,
        qos_profile: QoSProfile | None = None,
        callback_group: CallbackGroup | None = None,
    ) -> None:
        self._node = node
        self._qos_profile = (
            qos_profile if qos_profile is not None else _make_default_qos_profile()
        )
        self._callback_group = callback_group
        self._publishers_by_topic: dict[str, tuple[type, Any]] = {}

    def resolve_topic_types(self, topic: str) -> list[str]:
        normalized_topic = normalize_ros_topic(topic)
        if not normalized_topic:
            return []

        try:
            topics = self._node.get_topic_names_and_types()
        except Exception as exc:
            logger.warning(
                "Failed to get ROS topic names/types for type resolution: %s",
                exc,
                exc_info=True,
            )
            return []

        for name, types in topics:
            if name == normalized_topic:
                return list(types)
        return []

    def publish_message(self, topic: str, message: object | None) -> None:
        if message is None:
            logger.warning("Cannot publish ROS message: message is None.")
            return

        ros_topic = normalize_ros_topic(topic)
        if not ros_topic:
            logger.warning("Cannot publish ROS message: invalid topic=%s", topic)
            return

        msg_type = type(message)
        registered_publisher = self._publishers_by_topic.get(ros_topic)
        if registered_publisher is None:
            publisher = self._node.create_publisher(
                msg_type,
                ros_topic,
                self._qos_profile,
                callback_group=self._callback_group,
            )
            self._publishers_by_topic[ros_topic] = (msg_type, publisher)
        else:
            registered_type, publisher = registered_publisher
            if registered_type is not msg_type:
                logger.warning(
                    "Cannot publish to topic=%s: registered type mismatch expected=%s got=%s",
                    ros_topic,
                    registered_type,
                    msg_type,
                )
                return

        publisher.publish(message)

    def unregister_publisher(self, topic: str) -> bool:
        ros_topic = normalize_ros_topic(topic)
        if not ros_topic:
            logger.warning("Cannot unregister topic publisher: invalid topic=%s", topic)
            return False

        registered_publisher = self._publishers_by_topic.get(ros_topic)
        if registered_publisher is None:
            return False

        _, publisher = registered_publisher
        try:
            success = self._node.destroy_publisher(publisher)
        except Exception as exc:
            logger.warning(
                "Failed to destroy topic publisher for topic=%s: %s",
                ros_topic,
                exc,
                exc_info=True,
            )
            return False
        if not success:
            logger.warning("Failed to destroy topic publisher for topic=%s", ros_topic)
            return False

        self._publishers_by_topic.pop(ros_topic, None)
        return True


__all__ = ["RosPublisherRegistry"]
