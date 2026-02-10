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

from typing import Any
from unittest.mock import MagicMock

import pytest


class DummyMsg:
    pass


def make_subscription_node() -> MagicMock:
    node = MagicMock()
    node.create_subscription.return_value = MagicMock()
    node.destroy_subscription.return_value = True
    return node


def patch_get_message(
    monkeypatch: pytest.MonkeyPatch,
    *,
    module_path: str,
    message_type: type = DummyMsg,
) -> type:
    monkeypatch.setattr(f"{module_path}.get_message", lambda _name: message_type)
    return message_type


def patch_set_message_fields(
    monkeypatch: pytest.MonkeyPatch,
    *,
    module_path: str,
) -> MagicMock:
    setter = MagicMock()
    monkeypatch.setattr(f"{module_path}.set_message_fields", setter)
    return setter


def patch_message_to_ordereddict(
    monkeypatch: pytest.MonkeyPatch,
    *,
    module_path: str,
    payload: dict[str, Any],
) -> None:
    monkeypatch.setattr(f"{module_path}.message_to_ordereddict", lambda _msg: payload)


__all__ = [
    "DummyMsg",
    "make_subscription_node",
    "patch_get_message",
    "patch_message_to_ordereddict",
    "patch_set_message_fields",
]
