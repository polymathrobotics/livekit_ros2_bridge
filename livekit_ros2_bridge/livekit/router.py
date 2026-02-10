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
import asyncio
from concurrent.futures import Future
import functools
import json
import logging
import time
from collections.abc import Awaitable, Callable
from typing import Any, Protocol, TypeVar, cast

from livekit import rtc
from pydantic import ValidationError

from livekit_ros2_bridge.core.access import AccessDecision
from livekit_ros2_bridge.core.request_context import RequestContext, RequestSource
from livekit_ros2_bridge.core.names import normalize_ros_topic
from livekit_ros2_bridge.core.protocol import (
    PUBLISH_TOPIC,
    RPC_SERVICE_CALL,
    RPC_TOPIC_SUBSCRIBE,
    RPC_TOPIC_UNSUBSCRIBE,
    RPC_ERROR_FORBIDDEN,
    RPC_ERROR_INTERNAL,
    RPC_ERROR_INVALID_REQUEST,
    RPC_ERROR_UNAUTHORIZED,
    LivekitRosSubscriptionInfo,
    LivekitRpcCallServiceRequest,
    LivekitRpcCallServiceResponse,
    LivekitRpcServiceResponse,
    LivekitRpcSubscribeRequest,
    LivekitRpcSubscribeResponse,
    LivekitRpcSubscriptionStatus,
    LivekitRpcUnsubscribeRequest,
)
from livekit_ros2_bridge.core.telemetry import (
    IngressPublishTelemetryEvent,
    NullTelemetry,
    RpcTelemetryEvent,
    Telemetry,
)
from livekit_ros2_bridge.livekit.cancellable_workflow import (
    run_cancellable_work,
)
from livekit_ros2_bridge.livekit.utils import (
    decode_payload_text,
    extract_participant_id,
    extract_publish_metadata,
    extract_requester_id,
    get_participant_id_from_packet,
    normalize_livekit_topic,
)
from livekit_ros2_bridge.core.dispatcher import WorkDispatcher
from livekit_ros2_bridge.ros2.service_caller import (
    ServiceCallResult,
    ServiceCallError,
)

logger = logging.getLogger(__name__)

T = TypeVar("T")
RequestT = TypeVar("RequestT")

_RPC_ERROR_RULES: tuple[tuple[tuple[type[Exception], ...], int], ...] = (
    ((PermissionError,), RPC_ERROR_FORBIDDEN),
    (
        (
            ValueError,
            LookupError,
            KeyError,
        ),
        RPC_ERROR_INVALID_REQUEST,
    ),
    (
        (
            TimeoutError,
            ServiceCallError,
        ),
        RPC_ERROR_INTERNAL,
    ),
)


def rpc_error_handler(method: str, translate: Callable[[Exception], rtc.RpcError]):
    def decorator(func: Callable[..., Awaitable[str]]):
        @functools.wraps(func)
        async def wrapper(self: "LivekitRouter", data: rtc.RpcInvocationData) -> str:
            start = time.monotonic()
            requester_id = extract_requester_id(data)

            def _record(*, ok: bool, code: int | None) -> None:
                elapsed_ms = int((time.monotonic() - start) * 1000)
                self._record_rpc(
                    method=method,
                    ok=ok,
                    code=code,
                    latency_ms=elapsed_ms,
                    requester_id=requester_id,
                )

            try:
                response_json = await func(self, data)
            except asyncio.CancelledError:
                raise
            except rtc.RpcError as exc:
                _record(ok=False, code=getattr(exc, "code", None))
                raise
            except Exception as exc:
                translated = translate(exc)
                _record(ok=False, code=getattr(translated, "code", None))
                raise translated from exc
            else:
                _record(ok=True, code=None)
                return response_json

        return wrapper

    return decorator


def _translate_rpc_error(action: str) -> Callable[[Exception], rtc.RpcError]:
    def translate(exc: Exception) -> rtc.RpcError:
        for exc_types, code in _RPC_ERROR_RULES:
            if isinstance(exc, exc_types):
                return rtc.RpcError(code, str(exc))

        logger.error("RPC %s failed: %s", action, exc, exc_info=True)
        return rtc.RpcError(
            RPC_ERROR_INTERNAL, f"Internal error handling {action} request"
        )

    return translate


class _Publisher(Protocol):
    def handle_publish_payload(
        self,
        payload: Any,
        *,
        ctx: RequestContext | None = None,
    ) -> AccessDecision: ...


class _Subscriber(Protocol):
    def shutdown(self) -> None: ...
    def remove_participant(self, requester_id: str) -> None: ...
    def clear_livekit_requesters(self) -> None: ...

    def subscribe_request(
        self, ctx: RequestContext, rpc_request: LivekitRpcSubscribeRequest
    ) -> tuple[LivekitRosSubscriptionInfo, LivekitRpcSubscriptionStatus]: ...

    def unsubscribe_request(
        self, ctx: RequestContext, rpc_request: LivekitRpcUnsubscribeRequest
    ) -> tuple[LivekitRosSubscriptionInfo, LivekitRpcSubscriptionStatus]: ...


class _ServiceCaller(Protocol):
    def start_call(
        self,
        ctx: RequestContext,
        rpc_request: LivekitRpcCallServiceRequest,
        *,
        on_complete: Callable[[ServiceCallResult | Exception], None],
    ) -> str: ...

    def cancel_call(self, call_id: str, *, reason: str | None = None) -> bool: ...

    def shutdown(self) -> None: ...


class LivekitRouter:
    """Route LiveKit RPC and data-channel traffic for the ROS bridge."""

    def __init__(
        self,
        ros_publisher: _Publisher,
        subscriber: _Subscriber,
        service_caller: _ServiceCaller | None = None,
        *,
        work_dispatcher: WorkDispatcher,
        telemetry: Telemetry | None = None,
        dispatch_timeout_s: float = 5.0,
        service_default_timeout_ms: int = 2000,
        service_timeout_overhead_s: float = 0.5,
    ) -> None:
        self._subscriber = subscriber
        self._publisher = ros_publisher
        self._service_caller = service_caller
        self._work_dispatcher = work_dispatcher
        self._telemetry = telemetry or NullTelemetry()
        self._dispatch_timeout_s = float(dispatch_timeout_s)
        self._service_default_timeout_ms = max(int(service_default_timeout_ms), 0)
        self._service_timeout_overhead_s = max(float(service_timeout_overhead_s), 0.0)
        self._rpc_registered_for: int | None = None

    def shutdown(self) -> None:
        self._subscriber.shutdown()
        self._rpc_registered_for = None

    _RpcHandler = Callable[[rtc.RpcInvocationData], Awaitable[str]]

    def register_rpc_methods(
        self, local_participant: rtc.LocalParticipant | None
    ) -> None:
        if local_participant is None:
            logger.warning(
                "Cannot register RPC methods: local participant unavailable."
            )
            return

        participant_key = id(local_participant)
        if self._rpc_registered_for == participant_key:
            return

        self._register_rpc_method(
            local_participant,
            RPC_TOPIC_SUBSCRIBE,
            cast(LivekitRouter._RpcHandler, self._handle_rpc_subscribe),
        )
        self._register_rpc_method(
            local_participant,
            RPC_TOPIC_UNSUBSCRIBE,
            cast(LivekitRouter._RpcHandler, self._handle_rpc_unsubscribe),
        )
        if self._service_caller is not None:
            self._register_rpc_method(
                local_participant,
                RPC_SERVICE_CALL,
                cast(LivekitRouter._RpcHandler, self._handle_rpc_call_service),
            )
        self._rpc_registered_for = participant_key

    def _register_rpc_method(
        self,
        local_participant: rtc.LocalParticipant,
        method_name: str,
        handler: _RpcHandler,
    ) -> None:
        try:
            decorator = cast(
                Callable[[LivekitRouter._RpcHandler], LivekitRouter._RpcHandler],
                local_participant.register_rpc_method(method_name),
            )
            decorator(handler)
        except TypeError:
            local_participant.register_rpc_method(method_name, handler)

    def handle_data_packet(self, data: rtc.DataPacket | None) -> None:
        if data is None:
            return

        ctx = RequestContext(
            requester_id=get_participant_id_from_packet(data),
            source=RequestSource.LIVEKIT_DATA,
        )
        topic = normalize_livekit_topic(getattr(data, "topic", None))

        if topic != PUBLISH_TOPIC:
            return

        payload = self._parse_publish_payload(data, topic=topic, ctx=ctx)
        if payload is None:
            return

        future = self._work_dispatcher.submit(
            lambda: self._publisher.handle_publish_payload(payload, ctx=ctx)
        )
        future.add_done_callback(
            functools.partial(
                self._on_publish_dispatch_done,
                topic=topic,
                ctx=ctx,
                payload=payload,
            )
        )

    def _parse_publish_payload(
        self,
        data: rtc.DataPacket,
        *,
        topic: str,
        ctx: RequestContext,
    ) -> dict[str, Any] | None:
        try:
            decoded = decode_payload_text(data.data)
            payload_raw = json.loads(decoded)
        except Exception as exc:
            logger.error(
                "Failed decoding LiveKit data packet for topic=%s: %s",
                topic,
                exc,
                exc_info=True,
            )
            self._record_ingress_publish(
                ctx,
                ok=False,
                reason="decode_error",
                payload=None,
            )
            return None

        if isinstance(payload_raw, dict):
            return cast(dict[str, Any], payload_raw)

        logger.warning(
            "Ignoring non-dict data payload of type %s",
            type(payload_raw),
        )
        self._record_ingress_publish(
            ctx,
            ok=False,
            reason="invalid_payload",
            payload=None,
        )
        return None

    def _on_publish_dispatch_done(
        self,
        done: Future[AccessDecision],
        *,
        topic: str,
        ctx: RequestContext,
        payload: dict[str, Any],
    ) -> None:
        try:
            decision = done.result()
        except Exception as exc:
            logger.error(
                "Failed handling LiveKit publish payload for topic=%s: %s",
                topic,
                exc,
                exc_info=True,
            )
            self._record_ingress_publish(
                ctx,
                ok=False,
                reason="exception",
                payload=payload,
            )
            return

        self._record_ingress_publish(
            ctx,
            ok=decision.ok,
            reason=decision.reason,
            payload=payload,
        )

    def remove_participant(self, participant: Any) -> None:
        participant_id: str | None
        if isinstance(participant, str):
            participant_id = participant
        else:
            participant_id = extract_participant_id(participant)
        if not participant_id:
            return
        self._work_dispatcher.submit_noresult(
            lambda: self._subscriber.remove_participant(participant_id)
        )

    def clear_livekit_requesters(self) -> None:
        self._rpc_registered_for = None
        self._work_dispatcher.submit_noresult(self._subscriber.clear_livekit_requesters)

    async def _await_dispatched_result(
        self,
        fn: Callable[[], T],
        *,
        timeout_s: float,
        timeout_message: str,
    ) -> T:
        future = asyncio.wrap_future(self._work_dispatcher.submit(fn))
        try:
            if timeout_s > 0:
                return await asyncio.wait_for(future, timeout=timeout_s)
            return await future
        except asyncio.TimeoutError as exc:
            raise rtc.RpcError(RPC_ERROR_INTERNAL, timeout_message) from exc

    async def _handle_subscription_rpc(
        self,
        data: rtc.RpcInvocationData,
        *,
        action: str,
        parse_request: Callable[[rtc.RpcInvocationData], RequestT],
        handle_request: Callable[
            [RequestContext, RequestT],
            tuple[LivekitRosSubscriptionInfo, LivekitRpcSubscriptionStatus],
        ],
    ) -> str:
        requester_id = self._require_requester_id(data)
        rpc_request = parse_request(data)
        ctx = RequestContext(
            requester_id=requester_id, source=RequestSource.LIVEKIT_RPC
        )
        timeout_message = (
            f"Timed out waiting for ROS executor to handle {action} request."
        )
        info, status = await self._await_dispatched_result(
            lambda: handle_request(ctx, rpc_request),
            timeout_s=self._dispatch_timeout_s,
            timeout_message=timeout_message,
        )
        response = LivekitRpcSubscribeResponse(
            ok=True,
            subscription=info,
            status=status,
        )
        return response.json(allow_nan=False)

    def _parse_rpc_request(
        self,
        payload: Any,
        parser: Callable[[Any], RequestT],
    ) -> RequestT:
        try:
            return parser(payload)
        except ValidationError as exc:
            raise rtc.RpcError(
                RPC_ERROR_INVALID_REQUEST,
                "Invalid request payload",
            ) from exc

    def _parse_subscribe_request(
        self, data: rtc.RpcInvocationData
    ) -> LivekitRpcSubscribeRequest:
        return self._parse_rpc_request(
            data.payload,
            lambda payload: LivekitRpcSubscribeRequest.parse_raw(payload),
        )

    def _parse_unsubscribe_request(
        self, data: rtc.RpcInvocationData
    ) -> LivekitRpcUnsubscribeRequest:
        return self._parse_rpc_request(
            data.payload,
            lambda payload: LivekitRpcUnsubscribeRequest.parse_raw(payload),
        )

    def _parse_call_service_request(
        self,
        data: rtc.RpcInvocationData,
    ) -> LivekitRpcCallServiceRequest:
        return self._parse_rpc_request(
            data.payload,
            lambda payload: LivekitRpcCallServiceRequest.parse_raw(payload),
        )

    def _service_timeout_s(self, rpc_timeout_ms: int | None) -> float:
        timeout_ms = rpc_timeout_ms
        if timeout_ms is None or timeout_ms <= 0:
            timeout_ms = self._service_default_timeout_ms
        timeout_s = max(float(timeout_ms) / 1000.0, 0.0)
        if timeout_s > 0:
            timeout_s += self._service_timeout_overhead_s
        return timeout_s

    async def _invoke_service_call(
        self,
        *,
        service_caller: _ServiceCaller,
        ctx: RequestContext,
        rpc_request: LivekitRpcCallServiceRequest,
        timeout_s: float,
    ) -> ServiceCallResult:
        return await run_cancellable_work(
            dispatcher=self._work_dispatcher,
            start_operation=lambda on_complete: service_caller.start_call(
                ctx,
                rpc_request,
                on_complete=on_complete,
            ),
            cancel_operation=lambda call_id, reason: service_caller.cancel_call(
                call_id, reason=reason
            ),
            timeout_s=timeout_s,
            timeout_error=TimeoutError("Service call timed out."),
            timeout_reason="LiveKit RPC timed out.",
            cancelled_reason="LiveKit RPC cancelled.",
        )

    @rpc_error_handler(RPC_TOPIC_SUBSCRIBE, _translate_rpc_error("subscribe"))
    async def _handle_rpc_subscribe(self, data: rtc.RpcInvocationData) -> str:
        return await self._handle_subscription_rpc(
            data,
            action="subscribe",
            parse_request=self._parse_subscribe_request,
            handle_request=self._subscriber.subscribe_request,
        )

    @rpc_error_handler(RPC_TOPIC_UNSUBSCRIBE, _translate_rpc_error("unsubscribe"))
    async def _handle_rpc_unsubscribe(self, data: rtc.RpcInvocationData) -> str:
        return await self._handle_subscription_rpc(
            data,
            action="unsubscribe",
            parse_request=self._parse_unsubscribe_request,
            handle_request=self._subscriber.unsubscribe_request,
        )

    @rpc_error_handler(RPC_SERVICE_CALL, _translate_rpc_error("service.call"))
    async def _handle_rpc_call_service(self, data: rtc.RpcInvocationData) -> str:
        service_caller = self._service_caller
        if service_caller is None:
            raise rtc.RpcError(
                RPC_ERROR_INTERNAL,
                "Service calls are not enabled on this bridge",
            )

        start = time.monotonic()
        requester_id = self._require_requester_id(data)
        rpc_request = self._parse_call_service_request(data)
        ctx = RequestContext(
            requester_id=requester_id, source=RequestSource.LIVEKIT_RPC
        )
        timeout_s = self._service_timeout_s(rpc_request.timeout_ms)
        result = await self._invoke_service_call(
            service_caller=service_caller,
            ctx=ctx,
            rpc_request=rpc_request,
            timeout_s=timeout_s,
        )
        elapsed_ms = int((time.monotonic() - start) * 1000)
        response = LivekitRpcCallServiceResponse(
            ok=True,
            service=LivekitRpcServiceResponse(
                name=result.service,
                type=result.service_type,
            ),
            response=result.response,
            elapsed_ms=elapsed_ms,
        )
        return response.json(allow_nan=False)

    def _require_requester_id(self, data: rtc.RpcInvocationData) -> str:
        candidate = extract_requester_id(data)
        if candidate is not None:
            return candidate
        raise rtc.RpcError(
            RPC_ERROR_UNAUTHORIZED,
            "caller_identity is required for this RPC",
        )

    def _record_rpc(
        self,
        *,
        method: str,
        ok: bool,
        code: int | None,
        latency_ms: int,
        requester_id: str | None,
    ) -> None:
        try:
            self._telemetry.emit(
                RequestContext(
                    requester_id=requester_id,
                    source=RequestSource.LIVEKIT_RPC,
                ),
                RpcTelemetryEvent(
                    method=method,
                    ok=ok,
                    code=code,
                    latency_ms=latency_ms,
                ),
            )
        except Exception:
            logger.debug("Telemetry.emit failed for rpc", exc_info=True)

    def _record_ingress_publish(
        self,
        ctx: RequestContext,
        *,
        ok: bool,
        reason: str | None,
        payload: dict[str, Any] | None,
    ) -> None:
        raw_topic, ros_type = extract_publish_metadata(payload)
        topic = normalize_ros_topic(raw_topic)
        try:
            self._telemetry.emit(
                ctx,
                IngressPublishTelemetryEvent(
                    ok=ok,
                    reason=reason,
                    topic=topic,
                    ros_type=ros_type,
                ),
            )
        except Exception:
            logger.debug("Telemetry.emit failed for ingress_publish", exc_info=True)


__all__ = [
    "LivekitRouter",
    "RPC_ERROR_FORBIDDEN",
    "RPC_ERROR_INTERNAL",
    "RPC_ERROR_INVALID_REQUEST",
    "RPC_ERROR_UNAUTHORIZED",
]
