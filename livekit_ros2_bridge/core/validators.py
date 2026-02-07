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
"""Custom validators used by generated bridge parameters."""

from __future__ import annotations

from collections.abc import Iterable
from typing import Protocol, TypeGuard, cast


class _NamedValueParam(Protocol):
    name: str
    value: object


def _is_named_value_param(param: object) -> TypeGuard[_NamedValueParam]:
    if not hasattr(param, "name") or not hasattr(param, "value"):
        return False
    return isinstance(getattr(param, "name"), str)


def _require_named_value_param(param: object) -> _NamedValueParam:
    if not _is_named_value_param(param):
        raise TypeError("Expected parameter object with .name (str) and .value.")
    return cast(_NamedValueParam, param)


def no_blank_entries(param: object) -> str:
    """Reject array entries that are empty or whitespace-only."""
    typed_param = _require_named_value_param(param)
    if not isinstance(typed_param.value, Iterable) or isinstance(
        typed_param.value, (str, bytes)
    ):
        raise TypeError(
            "Expected parameter object with .name (str) and iterable .value."
        )

    values = typed_param.value
    if values == [""]:
        return ""
    for i, value in enumerate(values):
        if not str(value).strip():
            return (
                f"'{typed_param.name}' contains an empty entry at index {i}. "
                'Use [""] for an empty list.'
            )
    return ""


def no_surrounding_whitespace(param: object) -> str:
    """Reject values with leading or trailing whitespace."""
    typed_param = _require_named_value_param(param)
    value = str(typed_param.value)
    if value != value.strip():
        return f"'{typed_param.name}' must not include leading or trailing whitespace."
    return ""
