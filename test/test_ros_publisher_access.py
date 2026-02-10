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

from unittest.mock import MagicMock
from typing import Any

import pytest

from test.support.ros_harness import (
    DummyMsg,
    patch_get_message,
    patch_set_message_fields,
)
from livekit_ros2_bridge.core.access import AccessOperation
from livekit_ros2_bridge.core.access_static import StaticAccessPolicy
from livekit_ros2_bridge.core.request_context import RequestContext, RequestSource
from livekit_ros2_bridge.core.telemetry import AccessDenyTelemetryEvent
from livekit_ros2_bridge.ros2.publisher import PublisherConfig, RosPublisher


class DummyRosPublisherRegistry:
    def __init__(self, *, types: dict[str, list[str]] | None = None) -> None:
        self.publish_message = MagicMock()
        self.unregister_publisher = MagicMock(return_value=True)
        self._types = types or {}

    def resolve_topic_types(self, topic: str) -> list[str]:
        return list(self._types.get(topic, []))


def _make_publisher(
    *,
    allowlist: list[str] | None = None,
    denylist: list[str] | None = None,
    telemetry: Any | None = None,
) -> tuple[RosPublisher, DummyRosPublisherRegistry]:
    access_policy = StaticAccessPolicy(
        publish_allow=allowlist or [],
        publish_deny=denylist or [],
    )
    config = PublisherConfig(max_topics=50)
    ros_publisher_registry = DummyRosPublisherRegistry(
        types={
            "/allowed": ["std_msgs/msg/String"],
            "/allowed/child": ["std_msgs/msg/String"],
        }
    )
    return (
        RosPublisher(
            ros_publisher_registry,
            config,
            access_policy=access_policy,
            telemetry=telemetry,
        ),
        ros_publisher_registry,
    )


@pytest.mark.parametrize(
    "allowlist,denylist,topic,should_publish",
    [
        (["/allowed"], [], "/allowed", True),
        (["/allowed"], [], "/blocked", False),
        (["/allowed"], ["/allowed"], "/allowed", False),
        (["/allowed/*"], [], "/allowed/child", True),
    ],
)
def test_publish_allowlist_and_denylist(
    monkeypatch: pytest.MonkeyPatch,
    allowlist: list[str],
    denylist: list[str],
    topic: str,
    should_publish: bool,
) -> None:
    publisher, ros_publisher_registry = _make_publisher(
        allowlist=allowlist,
        denylist=denylist,
    )

    patch_get_message(
        monkeypatch,
        module_path="livekit_ros2_bridge.ros2.publisher",
        message_type=DummyMsg,
    )
    patch_set_message_fields(
        monkeypatch,
        module_path="livekit_ros2_bridge.ros2.publisher",
    )

    payload: dict[str, Any] = {
        "topic": topic,
        "type": "std_msgs/msg/String",
        "msg": {"data": "hello"},
    }

    publisher.handle_publish_payload(payload)

    if should_publish:
        ros_publisher_registry.publish_message.assert_called_once()
    else:
        ros_publisher_registry.publish_message.assert_not_called()


def test_publish_payload_ignores_legacy_format(monkeypatch: pytest.MonkeyPatch) -> None:
    publisher, ros_publisher_registry = _make_publisher(allowlist=["/user/*"])

    patch_get_message(
        monkeypatch,
        module_path="livekit_ros2_bridge.ros2.publisher",
        message_type=DummyMsg,
    )
    patch_set_message_fields(
        monkeypatch,
        module_path="livekit_ros2_bridge.ros2.publisher",
    )

    publisher.handle_publish_payload({"twist": {"linear": {"x": 1.0}}})

    ros_publisher_registry.publish_message.assert_not_called()


def test_publish_rejects_type_mismatch(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    publisher, ros_publisher_registry = _make_publisher(allowlist=["/allowed"])

    patch_get_message(
        monkeypatch,
        module_path="livekit_ros2_bridge.ros2.publisher",
        message_type=DummyMsg,
    )
    patch_set_message_fields(
        monkeypatch,
        module_path="livekit_ros2_bridge.ros2.publisher",
    )

    payload: dict[str, Any] = {
        "topic": "/allowed",
        "type": "std_msgs/msg/Int32",
        "msg": {"data": 1},
    }

    publisher.handle_publish_payload(payload)

    ros_publisher_registry.publish_message.assert_not_called()


def test_publish_accepts_graph_type_and_caches(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    access_policy = StaticAccessPolicy(publish_allow=["/allowed"])
    ros_publisher_registry = DummyRosPublisherRegistry(
        types={"/allowed": ["std_msgs/msg/String"]}
    )
    original_resolve = ros_publisher_registry.resolve_topic_types
    resolve_mock = MagicMock(side_effect=original_resolve)
    monkeypatch.setattr(
        ros_publisher_registry,
        "resolve_topic_types",
        resolve_mock,
    )
    publisher = RosPublisher(
        ros_publisher_registry,
        PublisherConfig(max_topics=50),
        access_policy=access_policy,
    )

    patch_get_message(
        monkeypatch,
        module_path="livekit_ros2_bridge.ros2.publisher",
        message_type=DummyMsg,
    )
    patch_set_message_fields(
        monkeypatch,
        module_path="livekit_ros2_bridge.ros2.publisher",
    )

    payload: dict[str, Any] = {
        "topic": "/allowed",
        "type": "std_msgs/msg/String",
        "msg": {"data": "hello"},
    }

    publisher.handle_publish_payload(payload)
    publisher.handle_publish_payload(payload)

    assert resolve_mock.call_count == 1
    assert ros_publisher_registry.publish_message.call_count == 2


def test_publish_denied_emits_access_event() -> None:
    telemetry = MagicMock()
    publisher, ros_publisher_registry = _make_publisher(
        allowlist=["/allowed"],
        telemetry=telemetry,
    )
    ctx = RequestContext(
        requester_id="participant-1", source=RequestSource.LIVEKIT_DATA
    )

    decision = publisher.handle_publish_payload(
        {
            "topic": "/blocked",
            "type": "std_msgs/msg/String",
            "msg": {"data": "hello"},
        },
        ctx=ctx,
    )

    assert decision.ok is False
    assert decision.reason == "publish_denied"
    ros_publisher_registry.publish_message.assert_not_called()
    telemetry.emit.assert_called_once()
    emit_ctx, event = telemetry.emit.call_args.args
    assert emit_ctx == ctx
    assert isinstance(event, AccessDenyTelemetryEvent)
    assert event.kind == "access_deny"
    assert event.op == AccessOperation.PUBLISH
    assert event.resource == "/blocked"
    assert event.reason == "publish_denied"
