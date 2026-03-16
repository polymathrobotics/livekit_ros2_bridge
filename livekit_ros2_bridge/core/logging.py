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

from types import TracebackType
from typing import Protocol, TypeAlias

ExcInfoTuple: TypeAlias = tuple[
    type[BaseException] | None,
    BaseException | None,
    TracebackType | None,
]
ExcInfoLike: TypeAlias = bool | BaseException | ExcInfoTuple


class Logger(Protocol):
    def child(self, *parts: str) -> "Logger": ...

    def throttled(self, duration_sec: float) -> "Logger": ...

    def debug(
        self,
        message: str,
        *args: object,
        exc_info: ExcInfoLike = False,
    ) -> None: ...

    def info(
        self,
        message: str,
        *args: object,
        exc_info: ExcInfoLike = False,
    ) -> None: ...

    def warning(
        self,
        message: str,
        *args: object,
        exc_info: ExcInfoLike = False,
    ) -> None: ...

    def error(
        self,
        message: str,
        *args: object,
        exc_info: ExcInfoLike = False,
    ) -> None: ...


__all__ = ["ExcInfoLike", "Logger"]
