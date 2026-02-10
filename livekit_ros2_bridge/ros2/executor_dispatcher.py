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
import queue
from concurrent.futures import Future
from threading import Lock
from typing import Any, Callable, TypeVar

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

logger = logging.getLogger(__name__)

T = TypeVar("T")


class RosExecutorDispatcher:
    """Thread-safe dispatcher for running work on the ROS executor thread.

    Why guard conditions (vs a timer)?
    - Triggering a guard condition wakes the ROS executor immediately.
    - A periodic timer adds latency and forces a polling cadence even when idle.
    """

    def __init__(self, node: Node, *, callback_group: CallbackGroup) -> None:
        self._node = node
        self._queue: queue.SimpleQueue[tuple[Callable[[], Any], Future[Any]]] = (
            queue.SimpleQueue()
        )
        self._shutdown = False
        self._lock = Lock()

        self._guard = node.create_guard_condition(
            self._drain,
            callback_group=callback_group,
        )

    def submit(self, fn: Callable[[], T]) -> Future[T]:
        future: Future[T] = Future()

        with self._lock:
            if self._shutdown:
                future.set_exception(RuntimeError("ROS dispatcher is shut down."))
                return future
            self._queue.put((fn, future))
        self._trigger()
        return future

    def submit_noresult(self, fn: Callable[[], None]) -> None:
        future = self.submit(fn)

        def _log_if_failed(done: Future[object]) -> None:
            try:
                done.result()
            except Exception:
                logger.error("ROS dispatcher task failed", exc_info=True)

        future.add_done_callback(_log_if_failed)  # type: ignore[arg-type]

    def shutdown(self) -> None:
        with self._lock:
            self._shutdown = True
        while True:
            try:
                _, future = self._queue.get_nowait()
            except queue.Empty:
                return
            if not future.done():
                future.set_exception(RuntimeError("ROS dispatcher is shut down."))

    def _trigger(self) -> None:
        try:
            self._guard.trigger()
        except Exception:
            logger.debug("Failed triggering ROS guard condition", exc_info=True)

    def _drain(self) -> None:
        while True:
            try:
                fn, future = self._queue.get_nowait()
            except queue.Empty:
                return

            if not future.set_running_or_notify_cancel():
                continue

            try:
                result = fn()
            except Exception as exc:
                future.set_exception(exc)
            else:
                future.set_result(result)


__all__ = ["RosExecutorDispatcher"]
