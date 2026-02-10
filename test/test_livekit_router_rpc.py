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

from test.support.livekit_stubs import StubRpcError
from test.support.router_harness import (
    DummyServiceCaller,
    DummySubscriber,
    HangingDispatcher,
    HangingServiceCaller,
    RaisingSubscriber,
    make_router,
    make_rpc_data,
    make_rpc_handler,
)
from livekit_ros2_bridge.core.protocol import (
    RPC_SERVICE_CALL,
    RPC_TOPIC_SUBSCRIBE,
    RPC_TOPIC_UNSUBSCRIBE,
)
from livekit_ros2_bridge.core.request_context import RequestContext, RequestSource
from livekit_ros2_bridge.core.telemetry import RpcTelemetryEvent
from livekit_ros2_bridge.livekit.router import (
    RPC_ERROR_INTERNAL,
    RPC_ERROR_INVALID_REQUEST,
    RPC_ERROR_UNAUTHORIZED,
)


@pytest.mark.asyncio
async def test_rpc_subscribe_success() -> None:
    subscriber = DummySubscriber()
    _, handler = make_rpc_handler(action=RPC_TOPIC_SUBSCRIBE, subscriber=subscriber)
    data = make_rpc_data(
        {"topic": "/foo", "preferred_interval_ms": 100},
        caller_identity="participant-1",
    )

    response = await handler(data)
    payload = json.loads(response)

    assert payload["ok"] is True
    assert payload["subscription"]["topic"] == "/foo"
    assert payload["status"]["applied_interval_ms"] == 100
    assert subscriber.last_requester == "participant-1"


@pytest.mark.asyncio
async def test_rpc_unsubscribe_success() -> None:
    subscriber = DummySubscriber()
    _, handler = make_rpc_handler(action=RPC_TOPIC_UNSUBSCRIBE, subscriber=subscriber)
    data = make_rpc_data({"topic": "/foo"}, caller_identity="participant-1")

    response = await handler(data)
    payload = json.loads(response)

    assert payload["ok"] is True
    assert payload["subscription"]["topic"] == "/foo"
    assert subscriber.last_requester == "participant-1"


@pytest.mark.asyncio
async def test_rpc_subscribe_emits_rpc_telemetry_event_success() -> None:
    telemetry = MagicMock()
    _, handler = make_rpc_handler(
        action=RPC_TOPIC_SUBSCRIBE,
        telemetry=telemetry,
    )
    data = make_rpc_data(
        {"topic": "/foo", "preferred_interval_ms": 100},
        caller_identity="participant-1",
    )

    await handler(data)

    telemetry.emit.assert_called_once()
    emit_ctx, event = telemetry.emit.call_args.args
    assert isinstance(emit_ctx, RequestContext)
    assert emit_ctx.requester_id == "participant-1"
    assert emit_ctx.source == RequestSource.LIVEKIT_RPC
    assert isinstance(event, RpcTelemetryEvent)
    assert event.kind == "rpc"
    assert event.method == RPC_TOPIC_SUBSCRIBE
    assert event.ok is True
    assert event.code is None
    assert event.latency_ms >= 0


@pytest.mark.asyncio
async def test_rpc_subscribe_requires_caller_identity() -> None:
    _, handler = make_rpc_handler(action=RPC_TOPIC_SUBSCRIBE)
    data = make_rpc_data(
        {"topic": "/foo", "preferred_interval_ms": 100},
        caller_identity=None,
    )

    with pytest.raises(StubRpcError) as exc:
        await handler(data)

    assert exc.value.code == RPC_ERROR_UNAUTHORIZED


@pytest.mark.asyncio
async def test_rpc_subscribe_invalid_request_maps_to_invalid_request_code() -> None:
    _, handler = make_rpc_handler(action=RPC_TOPIC_SUBSCRIBE)
    data = make_rpc_data({}, caller_identity="participant-1")

    with pytest.raises(StubRpcError) as exc:
        await handler(data)

    assert exc.value.code == RPC_ERROR_INVALID_REQUEST


@pytest.mark.asyncio
async def test_rpc_subscribe_emits_rpc_telemetry_event_failure() -> None:
    telemetry = MagicMock()
    _, handler = make_rpc_handler(action=RPC_TOPIC_SUBSCRIBE, telemetry=telemetry)
    data = make_rpc_data({}, caller_identity="participant-1")

    with pytest.raises(StubRpcError):
        await handler(data)

    telemetry.emit.assert_called_once()
    emit_ctx, event = telemetry.emit.call_args.args
    assert isinstance(emit_ctx, RequestContext)
    assert emit_ctx.requester_id == "participant-1"
    assert emit_ctx.source == RequestSource.LIVEKIT_RPC
    assert isinstance(event, RpcTelemetryEvent)
    assert event.kind == "rpc"
    assert event.method == RPC_TOPIC_SUBSCRIBE
    assert event.ok is False
    assert event.code == RPC_ERROR_INVALID_REQUEST
    assert event.latency_ms >= 0


@pytest.mark.asyncio
async def test_rpc_call_service_timeout() -> None:
    service_caller = DummyServiceCaller()
    service_caller.raise_exc = TimeoutError("timeout")
    _, handler = make_rpc_handler(
        action=RPC_SERVICE_CALL,
        service_caller=service_caller,
    )
    data = make_rpc_data(
        {"service": "/foo", "request": {}},
        caller_identity="participant-1",
    )

    with pytest.raises(StubRpcError) as exc:
        await handler(data)

    assert exc.value.code == RPC_ERROR_INTERNAL


@pytest.mark.asyncio
async def test_rpc_call_service_returns_internal_when_service_calls_disabled() -> None:
    router = make_router(service_caller=None)
    data = make_rpc_data(
        {"service": "/foo", "request": {}},
        caller_identity="participant-1",
    )

    with pytest.raises(StubRpcError) as exc:
        await router._handle_rpc_call_service(data)

    assert exc.value.code == RPC_ERROR_INTERNAL


@pytest.mark.asyncio
async def test_rpc_subscribe_times_out_waiting_for_ros_executor() -> None:
    _, handler = make_rpc_handler(
        action=RPC_TOPIC_SUBSCRIBE,
        dispatcher=HangingDispatcher(),
        dispatch_timeout_s=0.1,
    )
    data = make_rpc_data(
        {"topic": "/foo", "preferred_interval_ms": 100},
        caller_identity="participant-1",
    )

    with pytest.raises(StubRpcError) as exc:
        await handler(data)

    assert exc.value.code == RPC_ERROR_INTERNAL


@pytest.mark.asyncio
async def test_rpc_call_service_times_out_and_cancels_pending_call() -> None:
    service_caller = HangingServiceCaller()
    _, handler = make_rpc_handler(
        action=RPC_SERVICE_CALL,
        service_caller=service_caller,
        service_timeout_overhead_s=0.05,
    )
    data = make_rpc_data(
        {"service": "/foo", "request": {}, "timeout_ms": 100},
        caller_identity="participant-1",
    )

    with pytest.raises(StubRpcError) as exc:
        await handler(data)

    assert exc.value.code == RPC_ERROR_INTERNAL
    assert service_caller.cancelled[0][0] == "call-1"


@pytest.mark.asyncio
async def test_rpc_unexpected_error_translates_to_internal() -> None:
    _, handler = make_rpc_handler(
        action=RPC_TOPIC_SUBSCRIBE,
        subscriber=RaisingSubscriber(),
    )
    data = make_rpc_data(
        {"topic": "/foo", "preferred_interval_ms": 100},
        caller_identity="participant-1",
    )

    with pytest.raises(StubRpcError) as exc:
        await handler(data)

    assert exc.value.code == RPC_ERROR_INTERNAL
