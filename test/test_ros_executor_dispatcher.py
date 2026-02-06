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

from collections.abc import Callable

import pytest

from livekit_ros2_bridge.ros2 import executor_dispatcher as executor_dispatcher_module
from livekit_ros2_bridge.ros2.executor_dispatcher import RosExecutorDispatcher


class DummyGuard:
    def __init__(self) -> None:
        self.trigger_calls = 0
        self.raise_on_trigger = False

    def trigger(self) -> None:
        self.trigger_calls += 1
        if self.raise_on_trigger:
            raise RuntimeError("guard trigger failed")


class DummyNode:
    def __init__(self) -> None:
        self.guard = DummyGuard()
        self.drain_callback: Callable[[], None] | None = None
        self.callback_group: object | None = None

    def create_guard_condition(
        self,
        callback: Callable[[], None],
        *,
        callback_group: object | None = None,
    ) -> DummyGuard:
        self.drain_callback = callback
        self.callback_group = callback_group
        return self.guard


def test_submit_and_drain_runs_work() -> None:
    node = DummyNode()
    dispatcher = RosExecutorDispatcher(node, callback_group=object())

    future = dispatcher.submit(lambda: "ok")

    assert node.guard.trigger_calls == 1
    assert future.done() is False

    dispatcher._drain()

    assert future.result() == "ok"


def test_submit_noresult_logs_failures(monkeypatch: pytest.MonkeyPatch) -> None:
    node = DummyNode()
    dispatcher = RosExecutorDispatcher(node, callback_group=object())
    error_calls: list[tuple[str, object]] = []

    def _boom() -> None:
        raise RuntimeError("boom")

    def _record_error(message: str, *args: object, **kwargs: object) -> None:
        error_calls.append((message, kwargs.get("exc_info")))

    monkeypatch.setattr(executor_dispatcher_module.logger, "error", _record_error)

    dispatcher.submit_noresult(_boom)
    dispatcher._drain()

    assert error_calls == [("ROS dispatcher task failed", True)]


def test_shutdown_marks_pending_and_future_submissions_as_failed() -> None:
    node = DummyNode()
    dispatcher = RosExecutorDispatcher(node, callback_group=object())

    pending = dispatcher.submit(lambda: "never-runs")
    dispatcher.shutdown()

    with pytest.raises(RuntimeError, match="ROS dispatcher is shut down"):
        pending.result()

    after_shutdown = dispatcher.submit(lambda: "also-never-runs")
    with pytest.raises(RuntimeError, match="ROS dispatcher is shut down"):
        after_shutdown.result()


def test_submit_handles_guard_trigger_failures() -> None:
    node = DummyNode()
    node.guard.raise_on_trigger = True
    dispatcher = RosExecutorDispatcher(node, callback_group=object())

    future = dispatcher.submit(lambda: "ok")

    # Work is still queued and runnable even if trigger failed.
    dispatcher._drain()
    assert future.result() == "ok"
