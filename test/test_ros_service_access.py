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

from dataclasses import dataclass
from typing import Any, Callable, cast

import pytest

from livekit_ros2_bridge.core.access import AccessOperation
from livekit_ros2_bridge.core.access_static import StaticAccessPolicy
from livekit_ros2_bridge.core.request_context import RequestContext, RequestSource
from livekit_ros2_bridge.core.protocol import LivekitRpcCallServiceRequest
from livekit_ros2_bridge.core.telemetry import AccessDenyTelemetryEvent
from livekit_ros2_bridge.ros2 import service_caller as service_caller_module
from livekit_ros2_bridge.ros2.service_caller import (
    ServiceCallResult,
    RosServiceCaller,
    ServiceConfig,
)


class DummyFuture:
    def __init__(
        self,
        response: Any = None,
        *,
        exc: Exception | None = None,
        done: bool = True,
    ) -> None:
        self._response = response
        self._exc = exc
        self._done = done
        self._callbacks: list[Callable[[DummyFuture], None]] = []

    def done(self) -> bool:
        return self._done

    def add_done_callback(self, callback: Callable[["DummyFuture"], None]) -> None:
        self._callbacks.append(callback)
        if self._done:
            callback(self)

    def set_result(self, response: Any) -> None:
        self._response = response
        self._exc = None
        self._done = True
        for callback in list(self._callbacks):
            callback(self)

    def set_exception(self, exc: Exception) -> None:
        self._exc = exc
        self._done = True
        for callback in list(self._callbacks):
            callback(self)

    def result(self) -> Any:
        if self._exc:
            raise self._exc
        return self._response


class DummyClient:
    def __init__(self, future: DummyFuture) -> None:
        self._future = future
        self.wait_calls: list[float] = []
        self.call_args: list[Any] = []

    def service_is_ready(self) -> bool:
        return True

    def wait_for_service(self, timeout_sec: float) -> bool:
        self.wait_calls.append(timeout_sec)
        return True

    def call_async(self, request: Any) -> DummyFuture:
        self.call_args.append(request)
        return self._future


class DummyNode:
    def __init__(
        self,
        service_types: list[tuple[str, list[str]]] | None = None,
        *,
        client: DummyClient | None = None,
    ) -> None:
        self._service_types = service_types or []
        self._client = client or DummyClient(DummyFuture())
        self.created: list[tuple[Any, str]] = []

    def get_service_names_and_types(self) -> list[tuple[str, list[str]]]:
        return self._service_types

    def create_client(
        self, srv_type: Any, srv_name: str, *, callback_group: Any | None = None
    ) -> Any:
        del callback_group
        self.created.append((srv_type, srv_name))
        return self._client

    def create_timer(
        self,
        timer_period_sec: float,
        callback: Callable[[], None],
        callback_group: Any | None = None,
    ) -> Any:
        del timer_period_sec, callback, callback_group
        return object()

    def destroy_timer(self, timer: Any) -> bool:
        del timer
        return True


class DummyService:
    class Request:
        pass

    class Response:
        pass


class DummyTelemetry:
    def __init__(self) -> None:
        self.calls: list[tuple[RequestContext | None, object]] = []

    def emit(self, ctx: RequestContext | None, event: object) -> None:
        self.calls.append((ctx, event))


def _patch_ros_helpers(monkeypatch: pytest.MonkeyPatch) -> None:
    def _get_service(_service_type: str) -> type[DummyService]:
        return DummyService

    monkeypatch.setattr(service_caller_module, "get_service", _get_service)

    def _set_message_fields(msg: Any, fields: dict[str, Any]) -> None:
        for key, value in fields.items():
            setattr(msg, key, value)

    monkeypatch.setattr(
        service_caller_module, "set_message_fields", _set_message_fields
    )

    def _message_to_ordereddict(resp: Any) -> dict[str, Any]:
        return {"ok": bool(getattr(resp, "ok", False))}

    monkeypatch.setattr(
        service_caller_module, "message_to_ordereddict", _message_to_ordereddict
    )


def test_service_call_denied_by_default() -> None:
    node = DummyNode()
    access_policy = StaticAccessPolicy(service_allow=[])
    telemetry = DummyTelemetry()
    caller = RosServiceCaller(
        node,
        ServiceConfig(),
        access_policy=access_policy,
        telemetry=cast(Any, telemetry),
    )
    ctx = RequestContext(requester_id="client-1", source=RequestSource.LIVEKIT_RPC)

    with pytest.raises(PermissionError):
        caller.start_call(
            ctx,
            LivekitRpcCallServiceRequest(
                service="/foo",
                request={},
                type="pkg/srv/Foo",
                timeout_ms=None,
            ),
            on_complete=lambda _outcome: None,
        )

    assert len(telemetry.calls) == 1
    emit_ctx, event = telemetry.calls[0]
    assert emit_ctx == ctx
    assert isinstance(event, AccessDenyTelemetryEvent)
    assert event.kind == "access_deny"
    assert event.op == AccessOperation.CALL_SERVICE
    assert event.resource == "/foo"
    assert event.reason == "call_service_denied"


@dataclass(frozen=True)
class _DummyResponse:
    ok: bool


def test_service_call_allowlist_allows(monkeypatch: pytest.MonkeyPatch) -> None:
    _patch_ros_helpers(monkeypatch)

    response = _DummyResponse(ok=True)
    node = DummyNode(
        service_types=[("/foo", ["pkg/srv/Foo"])],
        client=DummyClient(DummyFuture(response=response)),
    )
    access_policy = StaticAccessPolicy(service_allow=["/foo"])
    caller = RosServiceCaller(node, ServiceConfig(), access_policy=access_policy)
    ctx = RequestContext(requester_id="client-1", source=RequestSource.LIVEKIT_RPC)

    outcomes: list[object] = []
    caller.start_call(
        ctx,
        LivekitRpcCallServiceRequest(
            service="/foo",
            request={"value": 1},
            type="pkg/srv/Foo",
            timeout_ms=100,
        ),
        on_complete=lambda outcome: outcomes.append(outcome),
    )

    assert len(outcomes) == 1
    assert not isinstance(outcomes[0], Exception)
    result = cast(ServiceCallResult, outcomes[0])

    assert result.service == "/foo"
    assert result.service_type == "pkg/srv/Foo"
    assert result.response["ok"] is True


def test_service_call_requires_resolved_type_when_missing_request_type() -> None:
    node = DummyNode(service_types=[])
    access_policy = StaticAccessPolicy(service_allow=["/foo"])
    caller = RosServiceCaller(node, ServiceConfig(), access_policy=access_policy)
    ctx = RequestContext(requester_id="client-1", source=RequestSource.LIVEKIT_RPC)

    with pytest.raises(ValueError, match="No ROS service types found"):
        caller.start_call(
            ctx,
            LivekitRpcCallServiceRequest(
                service="/foo",
                request={},
                type=None,
                timeout_ms=100,
            ),
            on_complete=lambda _outcome: None,
        )


def test_service_call_rejects_ambiguous_graph_service_types() -> None:
    node = DummyNode(service_types=[("/foo", ["pkg/srv/Foo", "pkg/srv/Bar"])])
    access_policy = StaticAccessPolicy(service_allow=["/foo"])
    caller = RosServiceCaller(node, ServiceConfig(), access_policy=access_policy)
    ctx = RequestContext(requester_id="client-1", source=RequestSource.LIVEKIT_RPC)

    with pytest.raises(ValueError, match="Multiple ROS service types found"):
        caller.start_call(
            ctx,
            LivekitRpcCallServiceRequest(
                service="/foo",
                request={},
                type=None,
                timeout_ms=100,
            ),
            on_complete=lambda _outcome: None,
        )


def test_service_call_cancel_call_and_unknown_call_id(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _patch_ros_helpers(monkeypatch)

    future = DummyFuture(done=False)
    node = DummyNode(
        service_types=[("/foo", ["pkg/srv/Foo"])],
        client=DummyClient(future),
    )
    access_policy = StaticAccessPolicy(service_allow=["/foo"])
    caller = RosServiceCaller(node, ServiceConfig(), access_policy=access_policy)
    ctx = RequestContext(requester_id="client-1", source=RequestSource.LIVEKIT_RPC)

    outcomes: list[object] = []
    call_id = caller.start_call(
        ctx,
        LivekitRpcCallServiceRequest(
            service="/foo",
            request={"value": 1},
            type="pkg/srv/Foo",
            timeout_ms=100,
        ),
        on_complete=lambda outcome: outcomes.append(outcome),
    )

    assert caller.cancel_call("missing") is False
    assert caller.cancel_call(call_id, reason="manual cancel") is True
    assert len(outcomes) == 1
    assert isinstance(outcomes[0], Exception)
    assert "manual cancel" in str(outcomes[0])


def test_service_call_enforces_inflight_limit(monkeypatch: pytest.MonkeyPatch) -> None:
    _patch_ros_helpers(monkeypatch)

    future = DummyFuture(done=False)
    node = DummyNode(
        service_types=[("/foo", ["pkg/srv/Foo"])],
        client=DummyClient(future),
    )
    access_policy = StaticAccessPolicy(service_allow=["/foo"])
    caller = RosServiceCaller(
        node,
        ServiceConfig(max_inflight_per_participant=1),
        access_policy=access_policy,
    )
    ctx = RequestContext(requester_id="client-1", source=RequestSource.LIVEKIT_RPC)

    call_id = caller.start_call(
        ctx,
        LivekitRpcCallServiceRequest(
            service="/foo",
            request={},
            type="pkg/srv/Foo",
            timeout_ms=100,
        ),
        on_complete=lambda _outcome: None,
    )

    with pytest.raises(PermissionError, match="Requester service call limit reached"):
        caller.start_call(
            ctx,
            LivekitRpcCallServiceRequest(
                service="/foo",
                request={},
                type="pkg/srv/Foo",
                timeout_ms=100,
            ),
            on_complete=lambda _outcome: None,
        )

    assert caller.cancel_call(call_id, reason="cleanup") is True


def test_service_call_shutdown_completes_pending_calls(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _patch_ros_helpers(monkeypatch)

    future = DummyFuture(done=False)
    node = DummyNode(
        service_types=[("/foo", ["pkg/srv/Foo"])],
        client=DummyClient(future),
    )
    access_policy = StaticAccessPolicy(service_allow=["/foo"])
    caller = RosServiceCaller(node, ServiceConfig(), access_policy=access_policy)
    ctx = RequestContext(requester_id="client-1", source=RequestSource.LIVEKIT_RPC)
    outcomes: list[object] = []

    caller.start_call(
        ctx,
        LivekitRpcCallServiceRequest(
            service="/foo",
            request={},
            type="pkg/srv/Foo",
            timeout_ms=100,
        ),
        on_complete=lambda outcome: outcomes.append(outcome),
    )

    caller.shutdown()

    assert len(outcomes) == 1
    assert isinstance(outcomes[0], Exception)
    assert "shut down" in str(outcomes[0])
