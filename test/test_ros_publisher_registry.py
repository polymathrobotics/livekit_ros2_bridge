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

from typing import Any

from livekit_ros2_bridge.ros2.publisher_registry import RosPublisherRegistry


class MsgA:
    pass


class MsgB:
    pass


class DummyPublisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class DummyNode:
    def __init__(self) -> None:
        self.topic_names_and_types: list[tuple[str, list[str]]] = []
        self.raise_get_types = False
        self.create_calls: list[tuple[type, str]] = []
        self.destroy_calls: list[object] = []
        self.publishers: list[DummyPublisher] = []
        self.destroy_return = True
        self.raise_destroy = False

    def get_topic_names_and_types(self) -> list[tuple[str, list[str]]]:
        if self.raise_get_types:
            raise RuntimeError("cannot read graph")
        return self.topic_names_and_types

    def create_publisher(
        self,
        msg_type: type,
        topic: str,
        qos_profile: Any,
        *,
        callback_group: Any = None,
    ) -> DummyPublisher:
        del qos_profile, callback_group
        self.create_calls.append((msg_type, topic))
        publisher = DummyPublisher()
        self.publishers.append(publisher)
        return publisher

    def destroy_publisher(self, publisher: object) -> bool:
        self.destroy_calls.append(publisher)
        if self.raise_destroy:
            raise RuntimeError("destroy failed")
        return self.destroy_return


def test_resolve_topic_types_handles_invalid_and_missing_topics() -> None:
    node = DummyNode()
    node.topic_names_and_types = [("/foo", ["std_msgs/msg/String"])]
    publisher_registry = RosPublisherRegistry(node)

    assert publisher_registry.resolve_topic_types("") == []
    assert publisher_registry.resolve_topic_types("/missing") == []
    assert publisher_registry.resolve_topic_types("/foo") == ["std_msgs/msg/String"]


def test_resolve_topic_types_handles_graph_exceptions() -> None:
    node = DummyNode()
    node.raise_get_types = True
    publisher_registry = RosPublisherRegistry(node)

    assert publisher_registry.resolve_topic_types("/foo") == []


def test_publish_creates_and_reuses_registered_publisher() -> None:
    node = DummyNode()
    publisher_registry = RosPublisherRegistry(node)

    msg1 = MsgA()
    msg2 = MsgA()
    publisher_registry.publish_message("/foo", msg1)
    publisher_registry.publish_message("/foo", msg2)

    assert len(node.create_calls) == 1
    assert node.create_calls[0][1] == "/foo"
    assert node.publishers[0].messages == [msg1, msg2]


def test_publish_rejects_invalid_inputs_and_type_mismatch() -> None:
    node = DummyNode()
    publisher_registry = RosPublisherRegistry(node)

    publisher_registry.publish_message("/foo", MsgA())
    publisher_registry.publish_message("/foo", MsgB())
    publisher_registry.publish_message("", MsgA())
    publisher_registry.publish_message("/foo", None)

    assert len(node.create_calls) == 1
    assert len(node.publishers[0].messages) == 1


def test_unregister_publisher_handles_success_failure_and_invalid_topic() -> None:
    node = DummyNode()
    publisher_registry = RosPublisherRegistry(node)

    publisher_registry.publish_message("/foo", MsgA())

    assert publisher_registry.unregister_publisher("/foo") is True
    assert publisher_registry.unregister_publisher("/foo") is False
    assert publisher_registry.unregister_publisher("") is False


def test_unregister_publisher_handles_destroy_failures() -> None:
    node = DummyNode()
    publisher_registry = RosPublisherRegistry(node)
    publisher_registry.publish_message("/foo", MsgA())

    node.destroy_return = False
    assert publisher_registry.unregister_publisher("/foo") is False

    node.destroy_return = True
    node.raise_destroy = True
    assert publisher_registry.unregister_publisher("/foo") is False
