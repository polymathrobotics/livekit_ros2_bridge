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

import asyncio
from collections.abc import Callable
from concurrent.futures import Future
from typing import Any

import pytest

from livekit_ros2_bridge.livekit.cancellable_workflow import (
    run_cancellable_work,
)


class ManualDispatcher:
    def __init__(self) -> None:
        self._submitted: list[tuple[Callable[[], Any], Future[Any]]] = []
        self._submit_noresult: list[Callable[[], None]] = []

    def submit(self, fn: Callable[[], Any]) -> Future[Any]:
        future: Future[Any] = Future()
        self._submitted.append((fn, future))
        return future

    def submit_noresult(self, fn: Callable[[], None]) -> None:
        self._submit_noresult.append(fn)

    def run_next_submit(self) -> None:
        fn, future = self._submitted.pop(0)
        if future.cancelled():
            return
        try:
            future.set_result(fn())
        except Exception as exc:
            future.set_exception(exc)

    def run_submit_noresult(self) -> None:
        pending = list(self._submit_noresult)
        self._submit_noresult.clear()
        for fn in pending:
            fn()


class ImmediateDispatcher:
    def submit(self, fn: Callable[[], Any]) -> Future[Any]:
        future: Future[Any] = Future()
        try:
            future.set_result(fn())
        except Exception as exc:
            future.set_exception(exc)
        return future

    def submit_noresult(self, fn: Callable[[], None]) -> None:
        fn()


@pytest.mark.asyncio
async def test_timeout_before_start_completion_still_cancels_once() -> None:
    dispatcher = ManualDispatcher()
    cancel_calls: list[tuple[str, str | None]] = []

    def start_operation(on_complete: Callable[[str | Exception], None]) -> str:
        del on_complete
        return "op-1"

    def cancel_operation(operation_id: str, reason: str | None) -> bool:
        cancel_calls.append((operation_id, reason))
        return True

    task = asyncio.create_task(
        run_cancellable_work(
            dispatcher=dispatcher,
            start_operation=start_operation,
            cancel_operation=cancel_operation,
            timeout_s=0.01,
            timeout_error=TimeoutError("timed out"),
            timeout_reason="timeout",
            cancelled_reason="cancelled",
        )
    )

    with pytest.raises(TimeoutError):
        await task

    assert cancel_calls == []
    dispatcher.run_next_submit()
    await asyncio.sleep(0)
    dispatcher.run_submit_noresult()
    assert cancel_calls == [("op-1", "timeout")]


@pytest.mark.asyncio
async def test_task_cancellation_cancels_operation() -> None:
    dispatcher = ImmediateDispatcher()
    cancel_calls: list[tuple[str, str | None]] = []

    def start_operation(on_complete: Callable[[str | Exception], None]) -> str:
        del on_complete
        return "op-2"

    def cancel_operation(operation_id: str, reason: str | None) -> bool:
        cancel_calls.append((operation_id, reason))
        return True

    task = asyncio.create_task(
        run_cancellable_work(
            dispatcher=dispatcher,
            start_operation=start_operation,
            cancel_operation=cancel_operation,
            timeout_s=1.0,
            timeout_error=TimeoutError("timed out"),
            timeout_reason="timeout",
            cancelled_reason="cancelled",
        )
    )
    await asyncio.sleep(0)
    task.cancel()

    with pytest.raises(asyncio.CancelledError):
        await task

    assert cancel_calls == [("op-2", "cancelled")]


@pytest.mark.asyncio
async def test_duplicate_completion_callbacks_do_not_override_result() -> None:
    dispatcher = ImmediateDispatcher()
    on_complete_holder: list[Callable[[str | Exception], None]] = []

    def start_operation(on_complete: Callable[[str | Exception], None]) -> str:
        on_complete_holder.append(on_complete)
        return "op-3"

    def cancel_operation(operation_id: str, reason: str | None) -> bool:
        del operation_id, reason
        return True

    task = asyncio.create_task(
        run_cancellable_work(
            dispatcher=dispatcher,
            start_operation=start_operation,
            cancel_operation=cancel_operation,
            timeout_s=1.0,
            timeout_error=TimeoutError("timed out"),
            timeout_reason="timeout",
            cancelled_reason="cancelled",
        )
    )
    await asyncio.sleep(0)

    on_complete = on_complete_holder[0]
    on_complete("ok")
    on_complete(RuntimeError("late failure"))
    assert await task == "ok"
