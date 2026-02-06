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

from dataclasses import dataclass, field
from typing import Literal, Protocol

from livekit_ros2_bridge.core.access import AccessOperation
from livekit_ros2_bridge.core.request_context import RequestContext


@dataclass(frozen=True)
class RpcTelemetryEvent:
    kind: Literal["rpc"] = field(default="rpc", init=False)
    method: str
    ok: bool
    code: int | None
    latency_ms: int


@dataclass(frozen=True)
class AccessDenyTelemetryEvent:
    kind: Literal["access_deny"] = field(default="access_deny", init=False)
    op: AccessOperation
    resource: str
    reason: str | None


@dataclass(frozen=True)
class IngressPublishTelemetryEvent:
    kind: Literal["ingress_publish"] = field(default="ingress_publish", init=False)
    ok: bool
    reason: str | None
    topic: str
    ros_type: str | None


@dataclass(frozen=True)
class EgressMessageTelemetryEvent:
    kind: Literal["egress_message"] = field(default="egress_message", init=False)
    topic: str
    ros_type: str
    bytes_len: int
    destinations: int


TelemetryEvent = (
    RpcTelemetryEvent
    | AccessDenyTelemetryEvent
    | IngressPublishTelemetryEvent
    | EgressMessageTelemetryEvent
)


class Telemetry(Protocol):
    def emit(self, ctx: RequestContext | None, event: TelemetryEvent) -> None: ...


class NullTelemetry:
    def emit(self, ctx: RequestContext | None, event: TelemetryEvent) -> None:
        del ctx, event
        return None


__all__ = [
    "EgressMessageTelemetryEvent",
    "IngressPublishTelemetryEvent",
    "NullTelemetry",
    "AccessDenyTelemetryEvent",
    "RpcTelemetryEvent",
    "TelemetryEvent",
    "Telemetry",
]
