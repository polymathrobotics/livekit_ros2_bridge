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

from typing import Any, Mapping, cast
from unittest.mock import MagicMock

import pytest
from rclpy.qos import QoSProfile

from test.support.ros_harness import (
    make_subscription_node,
    patch_message_to_ordereddict,
)
from livekit_ros2_bridge.core.protocol import (
    LivekitRosSubscriptionInfo,
    RosSubscriptionReliability,
)
from livekit_ros2_bridge.core.telemetry import EgressMessageTelemetryEvent
from livekit_ros2_bridge.ros2.topic_subscription import RosTopicSubscription


class DummyRoomPublisher:
    def __init__(self) -> None:
        self.calls: list[dict[str, object]] = []

    def can_publish_data(self) -> bool:
        return True

    def publish_data(
        self,
        topic: str,
        payload: Mapping[str, Any],
        reliable: bool = True,
        *,
        destination_identities: list[str] | None = None,
    ) -> None:
        del reliable
        self.calls.append(
            {
                "topic": topic,
                "payload": payload,
                "destination_identities": destination_identities,
            }
        )


def test_bridge_subscription_targets_requesters(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    node = make_subscription_node()

    room_publisher = DummyRoomPublisher()
    patch_message_to_ordereddict(
        monkeypatch,
        module_path="livekit_ros2_bridge.ros2.topic_subscription",
        payload={"data": "hello"},
    )

    info = LivekitRosSubscriptionInfo(
        topic="/foo",
        type="std_msgs/msg/String",
        depth=10,
        reliability=RosSubscriptionReliability.RELIABLE,
    )
    sub = RosTopicSubscription(
        node=node,
        room_publisher=room_publisher,
        info=info,
        qos_profile=QoSProfile(depth=10),
        msg_type=object,
        callback_group=None,
        message_id_provider=lambda: "msg-1",
        requesters={"p1": 0, "p2": 0},
        telemetry=None,
    )

    sub._ros_callback(object())

    assert len(room_publisher.calls) == 1
    assert set(cast(list[str], room_publisher.calls[0]["destination_identities"])) == {
        "p1",
        "p2",
    }


def test_bridge_subscription_does_not_publish_without_requesters(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    node = make_subscription_node()

    room_publisher = DummyRoomPublisher()
    patch_message_to_ordereddict(
        monkeypatch,
        module_path="livekit_ros2_bridge.ros2.topic_subscription",
        payload={"data": "hello"},
    )

    info = LivekitRosSubscriptionInfo(
        topic="/foo",
        type="std_msgs/msg/String",
        depth=10,
        reliability=RosSubscriptionReliability.RELIABLE,
    )
    sub = RosTopicSubscription(
        node=node,
        room_publisher=room_publisher,
        info=info,
        qos_profile=QoSProfile(depth=10),
        msg_type=object,
        callback_group=None,
        message_id_provider=lambda: "msg-1",
        requesters={},
        telemetry=None,
    )

    sub._ros_callback(object())

    assert room_publisher.calls == []


def test_bridge_subscription_emits_egress_message_event(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    node = make_subscription_node()

    room_publisher = DummyRoomPublisher()
    telemetry = MagicMock()
    patch_message_to_ordereddict(
        monkeypatch,
        module_path="livekit_ros2_bridge.ros2.topic_subscription",
        payload={"data": "hello"},
    )

    info = LivekitRosSubscriptionInfo(
        topic="/foo",
        type="std_msgs/msg/String",
        depth=10,
        reliability=RosSubscriptionReliability.RELIABLE,
    )
    sub = RosTopicSubscription(
        node=node,
        room_publisher=room_publisher,
        info=info,
        qos_profile=QoSProfile(depth=10),
        msg_type=object,
        callback_group=None,
        message_id_provider=lambda: "msg-1",
        requesters={"p1": 0},
        telemetry=telemetry,
    )

    sub._ros_callback(object())

    telemetry.emit.assert_called_once()
    emit_ctx, event = telemetry.emit.call_args.args
    assert emit_ctx is None
    assert isinstance(event, EgressMessageTelemetryEvent)
    assert event.kind == "egress_message"
    assert event.topic == "/foo"
    assert event.ros_type == "std_msgs/msg/String"
    assert event.destinations == 1
    assert event.bytes_len > 0
