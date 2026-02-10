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

import json

import pytest

from livekit_ros2_bridge.core.extensions import load_symbol


def test_load_symbol_supports_module_colon_attr() -> None:
    assert load_symbol("json:loads") is json.loads


def test_load_symbol_supports_module_dot_attr() -> None:
    assert load_symbol("json.loads") is json.loads


def test_load_symbol_supports_nested_attr_paths() -> None:
    decoder = load_symbol("json:decoder.JSONDecoder")
    assert decoder is json.decoder.JSONDecoder


@pytest.mark.parametrize("path", ["", "json", "json:", "json..loads", "json:decoder."])
def test_load_symbol_rejects_invalid_paths(path: str) -> None:
    with pytest.raises((ValueError, ImportError)):
        load_symbol(path)
