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
from typing import Any

from livekit_ros2_bridge.ros2.ros_logging import RosLogger


@dataclass(frozen=True)
class LogRecord:
    level: str
    name: str
    message: str
    kwargs: dict[str, Any]


class DummyLogger:
    def __init__(
        self,
        name: str = "test",
        *,
        records: list[LogRecord] | None = None,
    ) -> None:
        self.name = name
        self.records = records if records is not None else []

    def get_child(self, name: str) -> "DummyLogger":
        child_name = f"{self.name}.{name}" if name else self.name
        return DummyLogger(child_name, records=self.records)

    def debug(self, message: str, **kwargs: Any) -> None:
        self._record("debug", message, **kwargs)

    def info(self, message: str, **kwargs: Any) -> None:
        self._record("info", message, **kwargs)

    def warning(self, message: str, **kwargs: Any) -> None:
        self._record("warning", message, **kwargs)

    def error(self, message: str, **kwargs: Any) -> None:
        self._record("error", message, **kwargs)

    def _record(self, level: str, message: str, **kwargs: Any) -> None:
        self.records.append(
            LogRecord(
                level=level,
                name=self.name,
                message=message,
                kwargs=dict(kwargs),
            )
        )


def make_test_logger(name: str) -> RosLogger:
    return RosLogger(DummyLogger(name))


__all__ = ["DummyLogger", "LogRecord", "make_test_logger"]
