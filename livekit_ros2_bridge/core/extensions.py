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

import importlib
from typing import Any


def load_symbol(path: str) -> Any:
    """Load a Python symbol from `module:attr` or `module.attr`."""
    token = str(path or "").strip()
    if not token:
        raise ValueError("Symbol path is required.")

    module_name: str
    attr_path: str
    if ":" in token:
        module_name, attr_path = token.split(":", 1)
    else:
        module_name, _, attr_path = token.rpartition(".")
        if not module_name:
            raise ValueError(
                "Invalid symbol path; expected 'module:attr' or 'module.attr'."
            )

    module = importlib.import_module(module_name)
    value: Any = module
    for part in attr_path.split("."):
        part = part.strip()
        if not part:
            raise ValueError(
                "Invalid symbol path; attribute path contains an empty segment."
            )
        try:
            value = getattr(value, part)
        except AttributeError as exc:
            raise ImportError(
                f"Failed to resolve symbol '{token}': missing attribute '{part}'."
            ) from exc
    return value


__all__ = ["load_symbol"]
