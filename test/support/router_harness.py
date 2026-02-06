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
from concurrent.futures import Future
from types import SimpleNamespace
from typing import Any, cast
from unittest.mock import MagicMock

from livekit_ros2_bridge.core.protocol import (
    LivekitRosSubscriptionInfo,
    LivekitRpcCallServiceRequest,
    LivekitRpcSubscribeRequest,
    LivekitRpcSubscriptionStatus,
    LivekitRpcUnsubscribeRequest,
    RosSubscriptionReliability,
)
from livekit_ros2_bridge.core.request_context import RequestContext
from livekit_ros2_bridge.livekit.router import LivekitRouter
from livekit_ros2_bridge.ros2.service_caller import ServiceCallResult


class DummySubscriber:
    def __init__(self) -> None:
        self.info = LivekitRosSubscriptionInfo(
            topic="/foo",
            type="std_msgs/msg/String",
            depth=10,
            reliability=RosSubscriptionReliability.RELIABLE,
        )
        self.status = LivekitRpcSubscriptionStatus(
            applied_interval_ms=100, requester_count=1
        )
        self.last_requester: str | None = None
        self.last_preferred_interval_ms: int | None = None
        self.cleared = False
        self.removed_participants: list[str] = []
        self.shutdown_calls = 0

    def shutdown(self) -> None:
        self.shutdown_calls += 1

    def subscribe_request(
        self, ctx: RequestContext, rpc_request: LivekitRpcSubscribeRequest
    ) -> tuple[LivekitRosSubscriptionInfo, LivekitRpcSubscriptionStatus]:
        self.last_requester = ctx.requester_id
        self.last_preferred_interval_ms = rpc_request.preferred_interval_ms
        return self.info, self.status

    def unsubscribe_request(
        self, ctx: RequestContext, rpc_request: LivekitRpcUnsubscribeRequest
    ) -> tuple[LivekitRosSubscriptionInfo, LivekitRpcSubscriptionStatus]:
        del rpc_request
        self.last_requester = ctx.requester_id
        return self.info, self.status

    def clear_livekit_requesters(self) -> None:
        self.cleared = True

    def remove_participant(self, requester_id: str) -> None:
        self.removed_participants.append(requester_id)


class RaisingSubscriber(DummySubscriber):
    def subscribe_request(
        self, ctx: RequestContext, rpc_request: LivekitRpcSubscribeRequest
    ) -> tuple[LivekitRosSubscriptionInfo, LivekitRpcSubscriptionStatus]:
        del ctx, rpc_request
        raise RuntimeError("boom")


class DummyServiceCaller:
    def __init__(self) -> None:
        self.last_request: LivekitRpcCallServiceRequest | None = None
        self.last_ctx: RequestContext | None = None
        self.raise_exc: Exception | None = None
        self.cancelled: list[tuple[str, str | None]] = []
        self._call_id = 0
        self.result = ServiceCallResult(
            service="/foo",
            service_type="pkg/srv/Foo",
            response={"ok": True},
        )

    def shutdown(self) -> None:
        return None

    def start_call(
        self,
        ctx: RequestContext,
        rpc_request: LivekitRpcCallServiceRequest,
        *,
        on_complete: Any,
    ) -> str:
        self.last_ctx = ctx
        self.last_request = rpc_request
        if self.raise_exc is not None:
            raise self.raise_exc
        self._call_id += 1
        call_id = f"call-{self._call_id}"
        on_complete(self.result)
        return call_id

    def cancel_call(self, call_id: str, *, reason: str | None = None) -> bool:
        self.cancelled.append((call_id, reason))
        return True


class HangingServiceCaller(DummyServiceCaller):
    def start_call(
        self,
        ctx: RequestContext,
        rpc_request: LivekitRpcCallServiceRequest,
        *,
        on_complete: Any,
    ) -> str:
        del on_complete
        self.last_ctx = ctx
        self.last_request = rpc_request
        self._call_id += 1
        return f"call-{self._call_id}"


class DummyLocalParticipant:
    def __init__(self) -> None:
        self.handlers: dict[str, Any] = {}
        self.register_calls: list[str] = []

    def register_rpc_method(self, method_name: str, handler: Any | None = None) -> Any:
        self.register_calls.append(method_name)
        if handler is not None:
            self.handlers[method_name] = handler
            return handler

        def decorator(fn: Any) -> Any:
            self.handlers[method_name] = fn
            return fn

        return decorator


class DummyDispatcher:
    def submit(self, fn: Any) -> Future[Any]:
        future: Future[Any] = Future()
        try:
            future.set_result(fn())
        except Exception as exc:
            future.set_exception(exc)
        return future

    def submit_noresult(self, fn: Any) -> None:
        fn()


class HangingDispatcher:
    def submit(self, fn: Any) -> Future[Any]:
        del fn
        return Future()

    def submit_noresult(self, fn: Any) -> None:
        del fn
        return None


class FailingDispatcher:
    def submit(self, fn: Any) -> Future[Any]:
        del fn
        future: Future[Any] = Future()
        future.set_exception(RuntimeError("dispatch failed"))
        return future

    def submit_noresult(self, fn: Any) -> None:
        fn()


_DEFAULT = object()


def make_router(
    *,
    subscriber: DummySubscriber | None = None,
    service_caller: DummyServiceCaller | None | object = _DEFAULT,
    publisher: Any | None = None,
    telemetry: Any | None = None,
    dispatcher: Any | None = None,
    dispatch_timeout_s: float = 5.0,
    service_timeout_overhead_s: float = 0.5,
) -> LivekitRouter:
    if publisher is None:
        publisher = MagicMock()
    if dispatcher is None:
        dispatcher = DummyDispatcher()
    resolved_service_caller: DummyServiceCaller | None
    if service_caller is _DEFAULT:
        resolved_service_caller = DummyServiceCaller()
    else:
        resolved_service_caller = cast(DummyServiceCaller | None, service_caller)
    return LivekitRouter(
        publisher,
        subscriber or DummySubscriber(),
        service_caller=resolved_service_caller,
        work_dispatcher=dispatcher,
        telemetry=telemetry,
        dispatch_timeout_s=dispatch_timeout_s,
        service_timeout_overhead_s=service_timeout_overhead_s,
    )


def make_rpc_handler(
    *,
    action: str,
    subscriber: DummySubscriber | None = None,
    service_caller: DummyServiceCaller | None | object = _DEFAULT,
    telemetry: Any | None = None,
    dispatcher: Any | None = None,
    dispatch_timeout_s: float = 5.0,
    service_timeout_overhead_s: float = 0.5,
) -> tuple[LivekitRouter, Any]:
    router = make_router(
        subscriber=subscriber,
        service_caller=service_caller,
        telemetry=telemetry,
        dispatcher=dispatcher,
        dispatch_timeout_s=dispatch_timeout_s,
        service_timeout_overhead_s=service_timeout_overhead_s,
    )
    local_participant = DummyLocalParticipant()
    router.register_rpc_methods(cast(Any, local_participant))
    return router, local_participant.handlers[action]


def make_rpc_data(payload: dict[str, Any], *, caller_identity: str | None) -> Any:
    return SimpleNamespace(
        payload=json.dumps(payload),
        caller_identity=caller_identity,
    )


__all__ = [
    "DummyDispatcher",
    "DummyLocalParticipant",
    "DummyServiceCaller",
    "DummySubscriber",
    "FailingDispatcher",
    "HangingDispatcher",
    "HangingServiceCaller",
    "RaisingSubscriber",
    "make_router",
    "make_rpc_data",
    "make_rpc_handler",
]
