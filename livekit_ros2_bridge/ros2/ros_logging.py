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
import traceback
from typing import Any

from livekit_ros2_bridge.core.logging import ExcInfoLike

_RCLPY_CALLSITE_REGISTERED = False


def _register_rclpy_internal_caller(logger: Any) -> None:
    """Make rclpy skip this adapter module when it records log callsites."""
    global _RCLPY_CALLSITE_REGISTERED
    if _RCLPY_CALLSITE_REGISTERED:
        return

    try:
        from rclpy.impl import rcutils_logger
    except ImportError:
        return

    if not isinstance(logger, rcutils_logger.RcutilsLogger):
        return

    module_path = os.path.realpath(__file__)
    if module_path not in rcutils_logger._internal_callers:
        rcutils_logger._internal_callers.append(module_path)
    _RCLPY_CALLSITE_REGISTERED = True


class RosLogger:
    """Small compatibility layer over rclpy loggers."""

    def __init__(
        self,
        logger: Any,
        *,
        throttle_duration_sec: float | None = None,
    ) -> None:
        _register_rclpy_internal_caller(logger)
        self._logger = logger
        self._throttle_duration_sec = throttle_duration_sec

    @property
    def name(self) -> str:
        return str(getattr(self._logger, "name", ""))

    def child(self, *parts: str) -> "RosLogger":
        logger = self._logger
        for part in parts:
            if not part:
                continue
            logger = logger.get_child(part)
        if logger is self._logger:
            return self
        return RosLogger(
            logger,
            throttle_duration_sec=self._throttle_duration_sec,
        )

    def throttled(self, duration_sec: float) -> "RosLogger":
        seconds = float(duration_sec)
        if seconds <= 0:
            return self
        return RosLogger(self._logger, throttle_duration_sec=seconds)

    def debug(
        self,
        message: str,
        *args: object,
        exc_info: ExcInfoLike = False,
    ) -> None:
        self._log("debug", message, *args, exc_info=exc_info)

    def info(
        self,
        message: str,
        *args: object,
        exc_info: ExcInfoLike = False,
    ) -> None:
        self._log("info", message, *args, exc_info=exc_info)

    def warning(
        self,
        message: str,
        *args: object,
        exc_info: ExcInfoLike = False,
    ) -> None:
        self._log("warning", message, *args, exc_info=exc_info)

    def error(
        self,
        message: str,
        *args: object,
        exc_info: ExcInfoLike = False,
    ) -> None:
        self._log("error", message, *args, exc_info=exc_info)

    def _log(
        self,
        level: str,
        message: str,
        *args: object,
        exc_info: ExcInfoLike = False,
    ) -> None:
        formatted = self._format_message(message, *args)
        if exc_info:
            formatted = self._append_traceback(formatted, exc_info)
        kwargs: dict[str, Any] = {}
        if self._throttle_duration_sec is not None:
            kwargs["throttle_duration_sec"] = self._throttle_duration_sec
        getattr(self._logger, level)(formatted, **kwargs)

    @staticmethod
    def _format_message(message: str, *args: object) -> str:
        text = str(message)
        if not args:
            return text
        try:
            return text % args
        except Exception:
            suffix = " ".join(repr(arg) for arg in args)
            return f"{text} {suffix}".rstrip()

    @staticmethod
    def _append_traceback(message: str, exc_info: ExcInfoLike) -> str:
        trace = RosLogger._format_traceback(exc_info)
        if not trace:
            return message
        return f"{message}\n{trace}"

    @staticmethod
    def _format_traceback(exc_info: ExcInfoLike) -> str:
        if exc_info is True:
            formatted = traceback.format_exc().rstrip()
            if formatted == "NoneType: None":
                return ""
            return formatted
        if isinstance(exc_info, BaseException):
            return "".join(
                traceback.format_exception(
                    type(exc_info),
                    exc_info,
                    exc_info.__traceback__,
                )
            ).rstrip()
        if isinstance(exc_info, tuple) and len(exc_info) == 3:
            return "".join(
                traceback.format_exception(
                    exc_info[0],
                    exc_info[1],
                    exc_info[2],
                )
            ).rstrip()
        return ""


__all__ = ["RosLogger"]
