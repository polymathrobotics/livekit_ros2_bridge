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

from livekit_ros2_bridge.core.access import AccessOperation, AccessResource
from livekit_ros2_bridge.core.access_static import StaticAccessPolicy
from livekit_ros2_bridge.core.request_context import RequestContext, RequestSource


def test_static_access_default_denies_when_allowlist_empty() -> None:
    access_policy = StaticAccessPolicy()
    ctx = RequestContext(requester_id="p1", source=RequestSource.LIVEKIT_RPC)

    assert (
        access_policy.authorize(
            ctx, AccessOperation.PUBLISH, AccessResource(name="/foo")
        ).ok
        is False
    )
    assert (
        access_policy.authorize(
            ctx,
            AccessOperation.SUBSCRIBE,
            AccessResource(name="/foo", ros_type="std_msgs/msg/String"),
        ).ok
        is False
    )
    assert (
        access_policy.authorize(
            ctx, AccessOperation.CALL_SERVICE, AccessResource(name="/foo")
        ).ok
        is False
    )


def test_static_access_denylist_wins() -> None:
    access_policy = StaticAccessPolicy(publish_allow=["/foo"], publish_deny=["/foo"])
    ctx = RequestContext(requester_id=None, source=RequestSource.LIVEKIT_DATA)

    decision = access_policy.authorize(
        ctx,
        AccessOperation.PUBLISH,
        AccessResource(name="/foo", ros_type="std_msgs/msg/String"),
    )
    assert decision.ok is False


@pytest.mark.parametrize(
    "allowlist,name,expected",
    [
        (["/allowed"], "/allowed", True),
        (["/allowed"], "/blocked", False),
        (["/allowed/*"], "/allowed/child", True),
    ],
)
def test_static_access_publish_allowlist(
    allowlist: list[str], name: str, expected: bool
) -> None:
    access_policy = StaticAccessPolicy(publish_allow=allowlist)
    ctx = RequestContext(requester_id=None, source=RequestSource.LIVEKIT_DATA)

    decision = access_policy.authorize(
        ctx,
        AccessOperation.PUBLISH,
        AccessResource(name=name, ros_type="std_msgs/msg/String"),
    )
    assert decision.ok is expected


def test_static_access_subscribe_type_denylist() -> None:
    access_policy = StaticAccessPolicy(
        subscribe_allow=["*"],
        subscribe_type_deny=["sensor_msgs/msg/Image"],
    )
    ctx = RequestContext(requester_id="p1", source=RequestSource.LIVEKIT_RPC)

    denied = access_policy.authorize(
        ctx,
        AccessOperation.SUBSCRIBE,
        AccessResource(name="/camera/image", ros_type="sensor_msgs/msg/Image"),
    )
    assert denied.ok is False
    assert denied.reason == "subscribe_type_denied"

    allowed = access_policy.authorize(
        ctx,
        AccessOperation.SUBSCRIBE,
        AccessResource(name="/camera/info", ros_type="sensor_msgs/msg/CameraInfo"),
    )
    assert allowed.ok is True
