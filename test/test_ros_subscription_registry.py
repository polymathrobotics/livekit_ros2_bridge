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

import pytest

from test.support.ros_harness import make_subscription_node, patch_get_message
from livekit_ros2_bridge.core.access import AccessOperation
from livekit_ros2_bridge.core.access_static import StaticAccessPolicy
from livekit_ros2_bridge.core.request_context import RequestContext, RequestSource
from livekit_ros2_bridge.core.protocol import LivekitRpcSubscribeRequest
from livekit_ros2_bridge.core.telemetry import AccessDenyTelemetryEvent, Telemetry
from livekit_ros2_bridge.ros2.subscription_registry import (
    RosSubscriptionRegistry,
    SubscriberConfig,
)


class DummyRoomPublisher:
    def can_publish_data(self) -> bool:
        return False

    def publish_data(self, *args: object, **kwargs: object) -> None:
        raise AssertionError("publish_data should not be called in these tests")


def _make_subscription_registry(
    monkeypatch: pytest.MonkeyPatch,
    *,
    allowlist: list[str] | None = None,
    telemetry: Telemetry | None = None,
) -> tuple[RosSubscriptionRegistry, MagicMock]:
    node = make_subscription_node()
    node.get_topic_names_and_types.return_value = [
        ("/foo", ["std_msgs/msg/String"]),
        ("/bar", ["std_msgs/msg/String"]),
    ]
    patch_get_message(
        monkeypatch,
        module_path="livekit_ros2_bridge.ros2.subscription_registry",
    )

    access_policy = StaticAccessPolicy(subscribe_allow=allowlist or [])
    config = SubscriberConfig()
    return (
        RosSubscriptionRegistry(
            node,
            DummyRoomPublisher(),
            config,
            access_policy=access_policy,
            telemetry=telemetry,
        ),
        node,
    )


def test_subscribe_normalizes_topic_and_tracks_requesters(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    subscription_registry, _ = _make_subscription_registry(
        monkeypatch,
        allowlist=["/foo"],
    )
    ctx1 = RequestContext(requester_id="p1", source=RequestSource.LIVEKIT_RPC)
    info1, status1 = subscription_registry.subscribe_request(
        ctx1,
        LivekitRpcSubscribeRequest(
            topic="foo/",
            preferred_interval_ms=200,
        ),
    )
    assert info1.topic == "/foo"
    assert status1.requester_count == 1
    ctx2 = RequestContext(requester_id="p2", source=RequestSource.LIVEKIT_RPC)
    _, status2 = subscription_registry.subscribe_request(
        ctx2,
        LivekitRpcSubscribeRequest(
            topic="/foo",
            preferred_interval_ms=0,
        ),
    )

    assert status2.applied_interval_ms == 0
    assert status2.requester_count == 2


def test_subscribe_denied_topic(monkeypatch: pytest.MonkeyPatch) -> None:
    subscription_registry, _ = _make_subscription_registry(
        monkeypatch,
        allowlist=["/foo"],
    )
    ctx = RequestContext(requester_id="participant", source=RequestSource.LIVEKIT_RPC)
    with pytest.raises(PermissionError):
        subscription_registry.subscribe_request(
            ctx,
            LivekitRpcSubscribeRequest(
                topic="/bar",
                preferred_interval_ms=200,
            ),
        )


def test_subscribe_denied_topic_emits_access_event(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    telemetry = MagicMock()
    subscription_registry, _ = _make_subscription_registry(
        monkeypatch,
        allowlist=["/foo"],
        telemetry=telemetry,
    )
    ctx = RequestContext(requester_id="participant", source=RequestSource.LIVEKIT_RPC)

    with pytest.raises(PermissionError):
        subscription_registry.subscribe_request(
            ctx,
            LivekitRpcSubscribeRequest(
                topic="/bar",
                preferred_interval_ms=200,
            ),
        )

    telemetry.emit.assert_called_once()
    emit_ctx, event = telemetry.emit.call_args.args
    assert emit_ctx == ctx
    assert isinstance(event, AccessDenyTelemetryEvent)
    assert event.kind == "access_deny"
    assert event.op == AccessOperation.SUBSCRIBE
    assert event.resource == "/bar"
    assert event.reason == "subscribe_denied"
