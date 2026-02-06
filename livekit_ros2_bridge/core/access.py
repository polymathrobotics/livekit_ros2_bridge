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
from enum import Enum
from typing import Protocol

from livekit_ros2_bridge.core.request_context import RequestContext


class AccessOperation(str, Enum):
    PUBLISH = "publish"
    SUBSCRIBE = "subscribe"
    CALL_SERVICE = "call_service"


@dataclass(frozen=True)
class AccessResource:
    name: str
    ros_type: str | None = None


@dataclass(frozen=True)
class AccessDecision:
    ok: bool
    reason: str | None = None  # stable for logs/metrics; not a user-facing contract


class AccessPolicy(Protocol):
    def authorize(
        self, ctx: RequestContext, op: AccessOperation, res: AccessResource
    ) -> AccessDecision: ...


__all__ = [
    "AccessOperation",
    "AccessPolicy",
    "AccessResource",
    "AccessDecision",
]
