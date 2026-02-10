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
"""Cross-thread orchestration for long-lived cancellable workflows.

This helper coordinates one cancellable lifecycle:
- start work on the dispatcher thread
- await completion on the current asyncio loop
- schedule cancellation on timeout/task cancellation

Bridge motivation:
- LiveKit service calls can outlive the asyncio task waiting on them.
- On timeout or caller cancellation, we still need to request ROS-side cancel
  so in-flight work can shut down cleanly instead of being orphaned.
"""

from __future__ import annotations

import asyncio
from collections.abc import Callable
from concurrent.futures import Future
from typing import TypeVar

from livekit_ros2_bridge.core.dispatcher import WorkDispatcher

ResultT = TypeVar("ResultT")


def _set_outcome_once(
    result_future: asyncio.Future[ResultT],
    outcome: ResultT | Exception,
) -> None:
    """Resolve a result future once, ignoring any duplicate completions."""

    if result_future.done():
        return
    if isinstance(outcome, Exception):
        result_future.set_exception(outcome)
        return
    result_future.set_result(outcome)


def _schedule_cancel(
    dispatcher: WorkDispatcher,
    *,
    cancel_operation: Callable[[str, str | None], bool],
    operation_id: str,
    reason: str,
) -> None:
    """Schedule cancellation on the dispatcher thread."""

    def run_cancel() -> None:
        cancel_operation(operation_id, reason)

    dispatcher.submit_noresult(run_cancel)


async def run_cancellable_work(
    *,
    dispatcher: WorkDispatcher,
    start_operation: Callable[[Callable[[ResultT | Exception], None]], str],
    cancel_operation: Callable[[str, str | None], bool],
    timeout_s: float,
    timeout_error: Exception,
    timeout_reason: str,
    cancelled_reason: str,
) -> ResultT:
    """Run one cancellable lifecycle across dispatcher+asyncio threads."""

    loop = asyncio.get_running_loop()
    result_future: asyncio.Future[ResultT] = loop.create_future()
    operation_id: str | None = None
    cancel_reason: str | None = None
    cancel_scheduled = False

    def on_complete(outcome: ResultT | Exception) -> None:
        # Completion may happen from the dispatcher thread.
        loop.call_soon_threadsafe(_set_outcome_once, result_future, outcome)

    start_raw_future: Future[str] = dispatcher.submit(
        lambda: start_operation(on_complete)
    )
    start_future: asyncio.Future[str] = asyncio.wrap_future(start_raw_future)

    def on_started(outcome: str | Exception) -> None:
        nonlocal operation_id, cancel_scheduled
        if isinstance(outcome, Exception):
            _set_outcome_once(result_future, outcome)
            return

        operation_id = outcome

        # Timeout/cancel may have happened before start returned an id.
        if cancel_reason is None or cancel_scheduled:
            return
        cancel_scheduled = True
        _schedule_cancel(
            dispatcher,
            cancel_operation=cancel_operation,
            operation_id=operation_id,
            reason=cancel_reason,
        )

    def on_started_raw(done: Future[str]) -> None:
        if done.cancelled():
            return

        try:
            outcome: str | Exception = done.result()
        except Exception as exc:
            outcome = exc

        # Completion may happen from the dispatcher thread.
        loop.call_soon_threadsafe(on_started, outcome)

    start_raw_future.add_done_callback(on_started_raw)

    def request_cancel(reason: str) -> None:
        nonlocal cancel_reason, cancel_scheduled
        result_future.cancel()
        cancel_reason = reason
        if operation_id is None or cancel_scheduled:
            return
        cancel_scheduled = True
        _schedule_cancel(
            dispatcher,
            cancel_operation=cancel_operation,
            operation_id=operation_id,
            reason=reason,
        )

    async def invoke() -> ResultT:
        nonlocal operation_id
        # Keep start future alive on timeout so late completion can still cancel.
        operation_id = await asyncio.shield(start_future)
        return await result_future

    try:
        if timeout_s > 0:
            return await asyncio.wait_for(invoke(), timeout=timeout_s)
        return await invoke()
    except asyncio.TimeoutError as exc:
        request_cancel(timeout_reason)
        raise timeout_error from exc
    except asyncio.CancelledError:
        request_cancel(cancelled_reason)
        raise


__all__ = ["run_cancellable_work"]
