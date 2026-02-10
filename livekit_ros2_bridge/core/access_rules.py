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

from collections.abc import Iterable

from livekit_ros2_bridge.core.names import normalize_ros_topic


def parse_allowlist(entries: Iterable[str] | None) -> tuple[bool, set[str]]:
    return _parse_entries(entries, allow_all_flag=True)


def parse_denylist(entries: Iterable[str] | None) -> set[str]:
    _, patterns = _parse_entries(entries, allow_all_flag=False)
    return patterns


def is_allowed(
    name: str, *, allow_all: bool, allowlist: set[str], denylist: set[str]
) -> bool:
    if allow_all:
        return not matches_any(name, denylist)
    if not allowlist:
        return False
    if matches_any(name, denylist):
        return False
    return matches_any(name, allowlist)


def matches_any(name: str, entries: set[str]) -> bool:
    return any(entry_matches(name, entry) for entry in entries)


def entry_matches(name: str, entry: str) -> bool:
    if entry.endswith("/*"):
        prefix = entry[:-2]
        return name.startswith(f"{prefix}/")
    return name == entry


def _parse_entries(
    entries: Iterable[str] | None, *, allow_all_flag: bool
) -> tuple[bool, set[str]]:
    allow_all = False
    patterns: set[str] = set()
    for entry in entries or []:
        token = entry.strip()
        if not token:
            continue
        if allow_all_flag and token == "*":
            allow_all = True
            continue
        normalized = normalize_ros_topic(token)
        if normalized:
            patterns.add(normalized)
    return allow_all, patterns


__all__ = [
    "entry_matches",
    "is_allowed",
    "matches_any",
    "parse_allowlist",
    "parse_denylist",
]
