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
"""Tests for custom generated-parameter validators."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from livekit_ros2_bridge.core.validators import (
    no_surrounding_whitespace,
    no_blank_entries,
)


def test_no_blank_entries_accepts_empty_list() -> None:
    param = SimpleNamespace(name="access.static.publish.allow", value=[])
    assert no_blank_entries(param) == ""


def test_no_blank_entries_accepts_generated_single_blank_sentinel() -> None:
    param = SimpleNamespace(name="access.static.publish.allow", value=[""])
    assert no_blank_entries(param) == ""
    assert param.value == [""]


def test_no_blank_entries_rejects_blank_string_item() -> None:
    param = SimpleNamespace(name="access.static.publish.allow", value=["", "/user/*"])
    message = no_blank_entries(param)
    assert "contains an empty entry" in message
    assert "access.static.publish.allow" in message


def test_no_blank_entries_rejects_whitespace_item() -> None:
    param = SimpleNamespace(name="access.static.service.allow", value=["   "])
    message = no_blank_entries(param)
    assert "contains an empty entry" in message


def test_no_surrounding_whitespace_rejects_surrounding_whitespace() -> None:
    param = SimpleNamespace(name="telemetry.factory", value=" foo.bar ")
    message = no_surrounding_whitespace(param)
    assert "leading or trailing whitespace" in message


def test_no_blank_entries_rejects_invalid_parameter_shape() -> None:
    with pytest.raises(TypeError, match="Expected parameter object"):
        no_blank_entries(object())


def test_no_surrounding_whitespace_rejects_invalid_parameter_shape() -> None:
    with pytest.raises(TypeError, match="Expected parameter object"):
        no_surrounding_whitespace(object())
