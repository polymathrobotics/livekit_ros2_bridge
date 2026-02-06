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
import pytest

from livekit_ros2_bridge.core import access_rules


def test_access_allow_all_allows_unless_denied() -> None:
    allow_all, allowlist = access_rules.parse_allowlist(["*"])
    denylist = access_rules.parse_denylist(["/blocked"])

    assert (
        access_rules.is_allowed(
            "/ok", allow_all=allow_all, allowlist=allowlist, denylist=denylist
        )
        is True
    )
    assert (
        access_rules.is_allowed(
            "/blocked",
            allow_all=allow_all,
            allowlist=allowlist,
            denylist=denylist,
        )
        is False
    )


def test_access_allowlist_required_when_not_allow_all() -> None:
    assert (
        access_rules.is_allowed(
            "/anything", allow_all=False, allowlist=set(), denylist=set()
        )
        is False
    )


@pytest.mark.parametrize(
    "entry,name,expected",
    [
        ("/allowed", "/allowed", True),
        ("/allowed", "/allowed/child", False),
        ("/allowed/*", "/allowed/child", True),
        ("/allowed/*", "/allowed", False),
    ],
)
def test_access_entry_matches(entry: str, name: str, expected: bool) -> None:
    assert access_rules.entry_matches(name, entry) is expected


def test_access_denylist_wins_over_allowlist() -> None:
    allow_all, allowlist = access_rules.parse_allowlist(["/foo"])
    denylist = access_rules.parse_denylist(["/foo"])

    assert allow_all is False
    assert (
        access_rules.is_allowed(
            "/foo", allow_all=allow_all, allowlist=allowlist, denylist=denylist
        )
        is False
    )
