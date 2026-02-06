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
from unittest.mock import MagicMock

import pytest

from test.support.router_harness import FailingDispatcher, make_router
from livekit_ros2_bridge.core.access import AccessDecision
from livekit_ros2_bridge.core.protocol import PUBLISH_TOPIC
from livekit_ros2_bridge.core.request_context import RequestContext, RequestSource
from livekit_ros2_bridge.core.telemetry import IngressPublishTelemetryEvent


@pytest.mark.parametrize(
    "topic,expect_publish",
    [
        (PUBLISH_TOPIC, True),
        ("", False),
        ("ros.topic.command", False),
        ("/teleop", False),
    ],
)
def test_handle_data_packet_routes(topic: str, expect_publish: bool) -> None:
    publisher = MagicMock()
    publisher.handle_publish_payload = MagicMock(return_value=AccessDecision(ok=True))
    router = make_router(publisher=publisher)

    data = MagicMock()
    data.topic = topic
    data.data = json.dumps({"twist": {}})

    router.handle_data_packet(data)

    if expect_publish:
        publisher.handle_publish_payload.assert_called_once()
        args, kwargs = publisher.handle_publish_payload.call_args
        assert args == ({"twist": {}},)
        assert kwargs["ctx"].source == RequestSource.LIVEKIT_DATA
    else:
        publisher.handle_publish_payload.assert_not_called()


def test_handle_data_packet_propagates_requester_id_from_metadata() -> None:
    publisher = MagicMock()
    publisher.handle_publish_payload = MagicMock(return_value=AccessDecision(ok=True))
    router = make_router(publisher=publisher)

    data = MagicMock()
    data.topic = PUBLISH_TOPIC
    data.participant_identity = "participant-123"
    data.data = json.dumps(
        {
            "topic": "/foo",
            "type": "std_msgs/msg/String",
            "msg": {"data": "hello"},
        }
    )

    router.handle_data_packet(data)

    publisher.handle_publish_payload.assert_called_once()
    _, kwargs = publisher.handle_publish_payload.call_args
    ctx = kwargs["ctx"]
    assert isinstance(ctx, RequestContext)
    assert ctx.requester_id == "participant-123"
    assert ctx.source == RequestSource.LIVEKIT_DATA


def test_handle_data_packet_emits_ingress_publish_telemetry_success() -> None:
    telemetry = MagicMock()
    publisher = MagicMock()
    publisher.handle_publish_payload = MagicMock(return_value=AccessDecision(ok=True))
    router = make_router(publisher=publisher, telemetry=telemetry)

    data = MagicMock()
    data.topic = PUBLISH_TOPIC
    data.participant_identity = "participant-123"
    data.data = json.dumps(
        {
            "topic": "/foo",
            "type": "std_msgs/msg/String",
            "msg": {"data": "hello"},
        }
    )

    router.handle_data_packet(data)

    telemetry.emit.assert_called_once()
    emit_ctx, event = telemetry.emit.call_args.args
    assert isinstance(emit_ctx, RequestContext)
    assert emit_ctx.requester_id == "participant-123"
    assert emit_ctx.source == RequestSource.LIVEKIT_DATA
    assert isinstance(event, IngressPublishTelemetryEvent)
    assert event.kind == "ingress_publish"
    assert event.ok is True
    assert event.reason is None
    assert event.topic == "/foo"
    assert event.ros_type == "std_msgs/msg/String"


def test_handle_data_packet_emits_ingress_publish_telemetry_decode_error() -> None:
    telemetry = MagicMock()
    publisher = MagicMock()
    publisher.handle_publish_payload = MagicMock(return_value=AccessDecision(ok=True))
    router = make_router(publisher=publisher, telemetry=telemetry)

    data = MagicMock()
    data.topic = PUBLISH_TOPIC
    data.participant_identity = "participant-123"
    data.data = "{"

    router.handle_data_packet(data)

    publisher.handle_publish_payload.assert_not_called()
    telemetry.emit.assert_called_once()
    emit_ctx, event = telemetry.emit.call_args.args
    assert isinstance(emit_ctx, RequestContext)
    assert emit_ctx.requester_id == "participant-123"
    assert emit_ctx.source == RequestSource.LIVEKIT_DATA
    assert isinstance(event, IngressPublishTelemetryEvent)
    assert event.kind == "ingress_publish"
    assert event.ok is False
    assert event.reason == "decode_error"
    assert event.topic == ""
    assert event.ros_type is None


def test_handle_data_packet_emits_ingress_publish_telemetry_invalid_payload() -> None:
    telemetry = MagicMock()
    publisher = MagicMock()
    publisher.handle_publish_payload = MagicMock(return_value=AccessDecision(ok=True))
    router = make_router(publisher=publisher, telemetry=telemetry)

    data = MagicMock()
    data.topic = PUBLISH_TOPIC
    data.participant_identity = "participant-123"
    data.data = "[]"

    router.handle_data_packet(data)

    publisher.handle_publish_payload.assert_not_called()
    telemetry.emit.assert_called_once()
    _, event = telemetry.emit.call_args.args
    assert isinstance(event, IngressPublishTelemetryEvent)
    assert event.kind == "ingress_publish"
    assert event.ok is False
    assert event.reason == "invalid_payload"


def test_handle_data_packet_emits_ingress_publish_telemetry_dispatch_exception() -> (
    None
):
    telemetry = MagicMock()
    publisher = MagicMock()
    router = make_router(
        publisher=publisher,
        telemetry=telemetry,
        dispatcher=FailingDispatcher(),
    )

    data = MagicMock()
    data.topic = PUBLISH_TOPIC
    data.participant_identity = "participant-123"
    data.data = json.dumps(
        {
            "topic": "/foo",
            "type": "std_msgs/msg/String",
            "msg": {"data": "hello"},
        }
    )

    router.handle_data_packet(data)

    telemetry.emit.assert_called_once()
    _, event = telemetry.emit.call_args.args
    assert isinstance(event, IngressPublishTelemetryEvent)
    assert event.kind == "ingress_publish"
    assert event.ok is False
    assert event.reason == "exception"
