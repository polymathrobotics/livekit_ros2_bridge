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

import math
import numbers
from typing import Any, cast


def sanitize_payload(payload: Any) -> Any:
    if isinstance(payload, dict):
        raw = cast(dict[object, Any], payload)
        return {str(key): sanitize_payload(value) for key, value in raw.items()}
    if isinstance(payload, (list, tuple)):
        return [sanitize_payload(value) for value in payload]
    if isinstance(payload, (bytes, bytearray)):
        return list(payload)
    if isinstance(payload, memoryview):
        return list(payload.tobytes())
    to_list = getattr(payload, "tolist", None)
    if callable(to_list):
        try:
            return sanitize_payload(to_list())
        except Exception:
            return payload
    if isinstance(payload, numbers.Real) and not isinstance(payload, numbers.Integral):
        try:
            is_finite = math.isfinite(payload)
        except (TypeError, ValueError, OverflowError):
            return payload
        if not is_finite:
            return None
    return payload


__all__ = ["sanitize_payload"]
