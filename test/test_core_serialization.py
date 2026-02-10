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
from typing import Any

from livekit_ros2_bridge.core.serialization import sanitize_payload


class _DummyToList:
    def __init__(self, value: Any) -> None:
        self._value = value

    def tolist(self) -> Any:
        return self._value


class _DummyBadToList:
    def tolist(self) -> Any:
        raise RuntimeError("cannot convert")


def test_sanitize_payload_converts_arrays_and_tolist() -> None:
    payload = {
        "data": b"abc",
        "nested": {"buf": bytearray(b"xy"), "mem": memoryview(b"za")},
        "values": _DummyToList([1, 2, 3]),
        "items": (1, (2, 3), [4, (5,)]),
    }
    sanitized = sanitize_payload(payload)

    assert sanitized["data"] == [97, 98, 99]
    assert sanitized["nested"]["buf"] == [120, 121]
    assert sanitized["nested"]["mem"] == [122, 97]
    assert sanitized["values"] == [1, 2, 3]
    assert sanitized["items"] == [1, [2, 3], [4, [5]]]


def test_sanitize_payload_non_finite_floats_become_none() -> None:
    payload = {"inf": float("inf"), "nan": float("nan"), "value": 1.5}
    sanitized = sanitize_payload(payload)

    assert sanitized["inf"] is None
    assert sanitized["nan"] is None
    assert sanitized["value"] == 1.5


def test_sanitize_payload_tolist_failure_returns_original_object() -> None:
    obj = _DummyBadToList()

    assert sanitize_payload(obj) is obj
