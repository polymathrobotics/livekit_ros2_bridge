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
"""Tests for telemetry factory contract validation in node wiring."""

# ruff: noqa: E402

from __future__ import annotations

from typing import Any, cast

import pytest

from livekit_ros2_bridge import node as node_module  # noqa: E402
from livekit_ros2_bridge.node import LivekitBridgeNode  # noqa: E402


class _EmitOnlyTelemetry:
    def emit(self, ctx: object | None, event: object) -> None:
        del ctx, event
        return None


class _MissingEmitTelemetry:
    pass


def test_required_telemetry_methods_expect_emit_only() -> None:
    assert node_module._REQUIRED_TELEMETRY_METHODS == ("emit",)


def test_validate_factory_result_accepts_emit_only_telemetry() -> None:
    cast(Any, LivekitBridgeNode._validate_factory_result)(
        object(),
        parameter_name="telemetry.factory",
        expected_type_name="Telemetry",
        created=_EmitOnlyTelemetry(),
        required_methods=("emit",),
    )


def test_validate_factory_result_rejects_missing_emit() -> None:
    with pytest.raises(TypeError, match=r"missing \.emit\(\)"):
        cast(Any, LivekitBridgeNode._validate_factory_result)(
            object(),
            parameter_name="telemetry.factory",
            expected_type_name="Telemetry",
            created=_MissingEmitTelemetry(),
            required_methods=("emit",),
        )
