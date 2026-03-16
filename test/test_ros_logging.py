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

import os
import sys

import pytest

from livekit_ros2_bridge.ros2.ros_logging import RosLogger
from test.support.logger_harness import DummyLogger


def test_adapter_formats_printf_style_messages_and_child_names() -> None:
    backend = DummyLogger("livekit_bridge")
    logger = RosLogger(backend).child("ros2", "publisher")

    logger.warning("topic=%s count=%s", "/foo", 2)

    assert len(backend.records) == 1
    assert backend.records[0].name == "livekit_bridge.ros2.publisher"
    assert backend.records[0].message == "topic=/foo count=2"


def test_adapter_appends_tracebacks_for_exc_info_true() -> None:
    backend = DummyLogger("livekit_bridge")
    logger = RosLogger(backend)

    try:
        raise RuntimeError("boom")
    except RuntimeError:
        logger.error("operation failed", exc_info=True)

    assert len(backend.records) == 1
    assert backend.records[0].message.startswith("operation failed\nTraceback")
    assert "RuntimeError: boom" in backend.records[0].message


def test_adapter_accepts_exception_tuple_and_throttle_filter() -> None:
    backend = DummyLogger("livekit_bridge")
    logger = RosLogger(backend).throttled(5.0)

    try:
        raise ValueError("bad")
    except ValueError:
        exc_info = sys.exc_info()

    logger.warning("warn throttled", exc_info=exc_info)

    assert len(backend.records) == 1
    assert "ValueError: bad" in backend.records[0].message
    assert backend.records[0].kwargs == {"throttle_duration_sec": 5.0}


def test_adapter_preserves_callsite_for_mixed_severity() -> None:
    rcutils_logger = pytest.importorskip("rclpy.impl.rcutils_logger")
    backend = rcutils_logger.RcutilsLogger(name="livekit_bridge")
    logger = RosLogger(backend)

    logger.info("info message")
    logger.warning("warning message")

    assert len(backend.contexts) == 2
    caller_paths = {caller_id.file_path for caller_id in backend.contexts}
    assert caller_paths == {os.path.realpath(__file__)}


def test_adapter_preserves_callsite_for_throttle_changes() -> None:
    rcutils_logger = pytest.importorskip("rclpy.impl.rcutils_logger")
    backend = rcutils_logger.RcutilsLogger(name="livekit_bridge")
    logger = RosLogger(backend)

    logger.warning("warning message")
    logger.throttled(1.0).warning("warning throttled")

    assert len(backend.contexts) == 2
    caller_paths = {caller_id.file_path for caller_id in backend.contexts}
    assert caller_paths == {os.path.realpath(__file__)}
