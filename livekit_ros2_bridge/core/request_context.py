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


class RequestSource(str, Enum):
    LIVEKIT_RPC = "livekit_rpc"
    LIVEKIT_DATA = "livekit_data"


@dataclass(frozen=True)
class RequestContext:
    """Request/participant metadata propagated through bridge operations."""

    requester_id: str | None
    source: RequestSource


__all__ = [
    "RequestContext",
    "RequestSource",
]
