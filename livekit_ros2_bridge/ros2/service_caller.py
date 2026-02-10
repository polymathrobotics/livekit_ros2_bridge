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
import time
from dataclasses import dataclass
from typing import Any, Callable, Protocol, cast

from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.timer import Timer
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_service

from livekit_ros2_bridge.core.access import (
    AccessOperation,
    AccessPolicy,
    AccessResource,
    AccessDecision,
)
from livekit_ros2_bridge.core.request_context import RequestContext
from livekit_ros2_bridge.core.names import normalize_ros_topic
from livekit_ros2_bridge.core.protocol import LivekitRpcCallServiceRequest
from livekit_ros2_bridge.core.serialization import sanitize_payload
from livekit_ros2_bridge.core.telemetry import (
    NullTelemetry,
    AccessDenyTelemetryEvent,
    Telemetry,
)

logger = logging.getLogger(__name__)

DEFAULT_TIMEOUT_MS = 2000
DEFAULT_MAX_INFLIGHT_PER_PARTICIPANT = 4
READY_CHECK_PERIOD_S = 0.1
MIN_TIMEOUT_PERIOD_S = 0.001
SERVICE_CALL_TIMEOUT_MESSAGE = "Service call timed out."


class _ServiceNode(Protocol):
    def get_service_names_and_types(self) -> list[tuple[str, list[str]]]: ...

    def create_client(
        self,
        srv_type: Any,
        srv_name: str,
        *,
        callback_group: CallbackGroup | None = None,
    ) -> Client: ...

    def create_timer(
        self,
        timer_period_sec: float,
        callback: Callable[[], None],
        callback_group: CallbackGroup | None = None,
    ) -> Timer: ...

    def destroy_timer(self, timer: Timer) -> bool: ...


class ServiceCallError(RuntimeError):
    """ROS service invocation failed after the request was accepted."""


@dataclass(frozen=True)
class ServiceCallResult:
    """Result payload returned to LiveKit after a service call completes."""

    service: str
    service_type: str
    response: dict[str, Any]


@dataclass(frozen=True)
class ServiceConfig:
    default_timeout_ms: int = DEFAULT_TIMEOUT_MS
    max_inflight_per_participant: int = DEFAULT_MAX_INFLIGHT_PER_PARTICIPANT


class RosServiceCaller:
    """Invoke ROS services on behalf of LiveKit RPC callers.

    Threading: all methods must be called from the ROS executor thread.
    """

    def __init__(
        self,
        node: _ServiceNode,
        config: ServiceConfig,
        *,
        access_policy: AccessPolicy,
        telemetry: Telemetry | None = None,
        callback_group: CallbackGroup | None = None,
    ) -> None:
        self._node = node
        self._callback_group = callback_group
        self._clients: dict[tuple[str, str], tuple[Any, Client]] = {}
        self._pending: dict[str, _PendingCall] = {}
        self._call_id = 0
        self._inflight: dict[str, int] = {}

        self._access_policy = access_policy
        self._telemetry = telemetry or NullTelemetry()

        self._default_timeout_ms = max(int(config.default_timeout_ms), 0)
        self._max_inflight_per_participant = max(
            int(config.max_inflight_per_participant), 0
        )

    def shutdown(self) -> None:
        pending = list(self._pending.values())
        self._pending.clear()
        for call in pending:
            call.complete(ServiceCallError("Service caller shut down."))

    def start_call(
        self,
        ctx: RequestContext,
        rpc_request: LivekitRpcCallServiceRequest,
        *,
        on_complete: Callable[[ServiceCallResult | Exception], None],
    ) -> str:
        service = normalize_ros_topic(rpc_request.service)
        if not service:
            raise ValueError("service is required")

        timeout_ms = rpc_request.timeout_ms
        if timeout_ms is None or timeout_ms <= 0:
            timeout_ms = self._default_timeout_ms
        deadline_s = time.monotonic() + (timeout_ms / 1000.0)

        service_type = self._resolve_service_type(service, rpc_request.type)
        self._authorize_or_raise(
            ctx,
            AccessOperation.CALL_SERVICE,
            AccessResource(name=service, ros_type=service_type),
        )

        srv_type, client = self._get_or_create_client(service, service_type)
        try:
            request_msg = srv_type.Request()
            set_message_fields(request_msg, rpc_request.request)
        except Exception as exc:
            raise ValueError(
                f"Invalid service request payload for {service}: {exc}"
            ) from exc

        requester_id = ctx.requester_id or ""
        self._acquire_inflight(requester_id)
        try:
            call_id = self._next_call_id()
            call = _PendingCall(
                call_id=call_id,
                node=self._node,
                callback_group=self._callback_group,
                requester_id=requester_id,
                release_inflight=self._release_inflight,
                on_complete=on_complete,
                service=service,
                service_type=service_type,
                client=client,
                request_msg=request_msg,
                deadline_s=deadline_s,
                remove_pending=self._remove_pending_call,
            )
            self._pending[call_id] = call
            call.start()
            return call_id
        except Exception:
            self._release_inflight(requester_id)
            raise

    def cancel_call(self, call_id: str, *, reason: str | None = None) -> bool:
        call = self._pending.get(call_id)
        if call is None:
            return False
        call.complete(ServiceCallError(reason or "Service call cancelled."))
        return True

    def _next_call_id(self) -> str:
        self._call_id += 1
        return f"call-{self._call_id}"

    def _remove_pending_call(self, call_id: str) -> None:
        self._pending.pop(call_id, None)

    def _authorize_or_raise(
        self,
        ctx: RequestContext,
        op: AccessOperation,
        res: AccessResource,
    ) -> AccessDecision:
        decision = self._access_policy.authorize(ctx, op, res)
        if decision.ok:
            return decision
        self._record_access_denial(ctx, op, res.name, decision)
        raise PermissionError(f"ROS service '{res.name}' not permitted.")

    def _record_access_denial(
        self,
        ctx: RequestContext,
        op: AccessOperation,
        resource: str,
        decision: AccessDecision,
    ) -> None:
        try:
            self._telemetry.emit(
                ctx,
                AccessDenyTelemetryEvent(
                    op=op,
                    resource=normalize_ros_topic(resource) or resource,
                    reason=decision.reason,
                ),
            )
        except Exception:
            logger.debug("Telemetry.emit failed for access denial", exc_info=True)

    def _resolve_service_type(self, service: str, service_type: str | None) -> str:
        if service_type is not None:
            candidate = service_type.strip()
            if candidate:
                return candidate

        types = self._resolve_graph_service_types(service)
        if not types:
            raise ValueError(f"No ROS service types found for service '{service}'.")
        if len(types) > 1:
            raise ValueError(
                f"Multiple ROS service types found for service '{service}': {', '.join(types)}."
            )
        return types[0]

    def _resolve_graph_service_types(self, service: str) -> list[str]:
        try:
            services = self._node.get_service_names_and_types()
        except Exception as exc:
            logger.error("Failed to resolve ROS service types: %s", exc, exc_info=True)
            return []

        for name, service_types in services:
            if name == service:
                return list(service_types)
        return []

    def _get_or_create_client(
        self,
        service: str,
        service_type: str,
    ) -> tuple[Any, Client]:
        key = (service, service_type)
        cached = self._clients.get(key)
        if cached is not None:
            return cached

        try:
            srv_type = cast(Any, get_service(service_type))
        except Exception as exc:
            raise ValueError(f"Unknown ROS service type: {service_type}") from exc

        try:
            client = self._node.create_client(
                srv_type,
                service,
                callback_group=self._callback_group,
            )
        except Exception as exc:
            raise ServiceCallError(
                f"Failed creating client for {service}: {exc}"
            ) from exc

        self._clients[key] = (srv_type, client)
        return srv_type, client

    def _acquire_inflight(self, requester_id: str) -> None:
        if not requester_id:
            return
        limit = self._max_inflight_per_participant
        current = self._inflight.get(requester_id, 0)
        if limit and current >= limit:
            raise PermissionError("Requester service call limit reached.")
        self._inflight[requester_id] = current + 1

    def _release_inflight(self, requester_id: str) -> None:
        if not requester_id:
            return
        current = self._inflight.get(requester_id, 0)
        if current <= 1:
            self._inflight.pop(requester_id, None)
            return
        self._inflight[requester_id] = current - 1


class _PendingCall:
    def __init__(
        self,
        *,
        call_id: str,
        node: _ServiceNode,
        callback_group: CallbackGroup | None,
        requester_id: str,
        release_inflight: Callable[[str], None],
        on_complete: Callable[[ServiceCallResult | Exception], None],
        service: str,
        service_type: str,
        client: Client,
        request_msg: Any,
        deadline_s: float,
        remove_pending: Callable[[str], None],
    ) -> None:
        self._call_id = call_id
        self._node = node
        self._callback_group = callback_group
        self._requester_id = requester_id
        self._release_inflight = release_inflight
        self._on_complete = on_complete
        self._service = service
        self._service_type = service_type
        self._client = client
        self._request_msg = request_msg
        self._deadline_s = deadline_s
        self._remove_pending = remove_pending

        self._ready_timer: Timer | None = None
        self._timeout_timer: Timer | None = None
        self._completed = False

    def start(self) -> None:
        if self._client.service_is_ready():
            self._begin_call()
            return

        self._ready_timer = self._node.create_timer(
            READY_CHECK_PERIOD_S,
            self._on_ready_timer,
            callback_group=self._callback_group,
        )
        self._on_ready_timer()

    def complete(self, outcome: ServiceCallResult | Exception) -> None:
        if self._completed:
            return
        self._completed = True

        self._remove_pending(self._call_id)
        self._release_inflight(self._requester_id)
        self._clear_timer("_ready_timer")
        self._clear_timer("_timeout_timer")

        try:
            self._on_complete(outcome)
        except Exception:
            logger.error("Service on_complete handler failed", exc_info=True)

    def _clear_timer(self, attr: str) -> None:
        timer = cast(Timer | None, getattr(self, attr))
        if timer is None:
            return
        setattr(self, attr, None)
        try:
            self._node.destroy_timer(timer)
        except Exception:
            logger.debug("Failed destroying ROS timer", exc_info=True)

    def _on_ready_timer(self) -> None:
        if self._completed:
            return
        if time.monotonic() >= self._deadline_s:
            self.complete(TimeoutError(f"Service '{self._service}' is unavailable."))
            return
        if not self._client.service_is_ready():
            return

        self._clear_timer("_ready_timer")
        self._begin_call()

    def _begin_call(self) -> None:
        if self._completed:
            return

        remaining_s = self._deadline_s - time.monotonic()
        if remaining_s <= 0:
            self.complete(self._timeout_error())
            return

        future = self._client.call_async(self._request_msg)
        if self._future_is_done(future):
            self._handle_future_done(future)
            return

        try:
            future.add_done_callback(self._handle_future_done)
        except Exception:
            pass

        self._timeout_timer = self._node.create_timer(
            max(remaining_s, MIN_TIMEOUT_PERIOD_S),
            self._on_timeout_timer,
            callback_group=self._callback_group,
        )

    def _on_timeout_timer(self) -> None:
        self._clear_timer("_timeout_timer")
        if self._completed:
            return
        self.complete(self._timeout_error())

    def _handle_future_done(self, future: Any) -> None:
        if self._completed:
            return

        try:
            response = future.result()
        except Exception as exc:
            self.complete(ServiceCallError(f"Service call failed: {exc}"))
            return

        if response is None:
            self.complete(ServiceCallError("Service call failed: empty response."))
            return

        try:
            payload = message_to_ordereddict(response)
        except Exception as exc:
            self.complete(
                ServiceCallError(f"Failed to serialize service response: {exc}")
            )
            return

        self.complete(
            ServiceCallResult(
                service=self._service,
                service_type=self._service_type,
                response=self._sanitize_response(payload),
            )
        )

    @staticmethod
    def _future_is_done(future: Any) -> bool:
        done_method = getattr(future, "done", None)
        return bool(done_method and done_method())

    @staticmethod
    def _timeout_error() -> TimeoutError:
        return TimeoutError(SERVICE_CALL_TIMEOUT_MESSAGE)

    @staticmethod
    def _sanitize_response(payload: Any) -> dict[str, Any]:
        sanitized = sanitize_payload(payload)
        if isinstance(sanitized, dict):
            return cast(dict[str, Any], sanitized)
        return {"response": sanitized}


__all__ = [
    "ServiceCallResult",
    "RosServiceCaller",
    "ServiceCallError",
    "ServiceConfig",
]
