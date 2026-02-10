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

from concurrent.futures import Future
from typing import Callable, Protocol, TypeVar

T = TypeVar("T")


class WorkDispatcher(Protocol):
    """Submit synchronous callables onto another execution context.

    This keeps the LiveKit router decoupled from the concrete ROS executor
    implementation that executes the submitted work.
    """

    def submit(self, fn: Callable[[], T]) -> Future[T]: ...
    def submit_noresult(self, fn: Callable[[], None]) -> None: ...


__all__ = ["WorkDispatcher"]
