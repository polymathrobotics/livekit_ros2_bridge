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

import sys
from types import ModuleType


class StubRpcError(Exception):
    def __init__(self, code: int, message: str) -> None:
        super().__init__(message)
        self.code = code


def install_livekit_stubs(*, include_api: bool = False) -> type[StubRpcError]:
    """Install minimal `livekit` stubs for unit tests.

    These tests focus on bridge behavior and should not require the real LiveKit SDK.
    """

    rtc_stub = ModuleType("livekit.rtc")
    setattr(rtc_stub, "RpcError", StubRpcError)
    setattr(rtc_stub, "RpcInvocationData", object)
    setattr(rtc_stub, "LocalParticipant", object)
    setattr(rtc_stub, "RemoteParticipant", object)
    setattr(rtc_stub, "DataPacket", object)
    setattr(rtc_stub, "Room", object)

    livekit_stub = ModuleType("livekit")
    setattr(livekit_stub, "rtc", rtc_stub)

    sys.modules["livekit"] = livekit_stub
    sys.modules["livekit.rtc"] = rtc_stub

    if include_api:
        api_stub = ModuleType("livekit.api")
        setattr(livekit_stub, "api", api_stub)
        sys.modules["livekit.api"] = api_stub

    return StubRpcError


__all__ = ["StubRpcError", "install_livekit_stubs"]
