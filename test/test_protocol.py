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
from pydantic import ValidationError
from typing import Any, cast

from livekit_ros2_bridge.core.protocol import (
    DATA_CONTENT_TYPE,
    LivekitRosMessageBody,
    LivekitRosSubscriptionInfo,
    LivekitRpcCallServiceRequest,
    LivekitRpcSubscribeRequest,
    LivekitRpcSubscribeResponse,
    LivekitRpcSubscriptionDataTransport,
    LivekitRpcSubscriptionStatus,
    LivekitRpcSubscriptionVideoTransport,
    RosSubscriptionReliability,
)


def test_call_service_request_requires_dict() -> None:
    with pytest.raises(ValidationError):
        LivekitRpcCallServiceRequest(service="example", request=cast(Any, "not-a-dict"))


def test_call_service_request_rejects_empty_type_when_provided() -> None:
    with pytest.raises(ValidationError):
        LivekitRpcCallServiceRequest(service="example", request={}, type=" ")


def test_subscribe_request_requires_topic() -> None:
    with pytest.raises(ValidationError):
        LivekitRpcSubscribeRequest(topic="", preferred_interval_ms=0)


def test_livekit_ros_message_body_uses_type_field() -> None:
    body = LivekitRosMessageBody(
        topic="/foo",
        type="std_msgs/msg/String",
        msg={"data": "hello"},
        content_type=DATA_CONTENT_TYPE,
    )
    payload = body.dict()

    assert payload["type"] == "std_msgs/msg/String"
    assert "ros_type" not in payload


def test_subscribe_response_requires_transport() -> None:
    with pytest.raises(ValidationError):
        LivekitRpcSubscribeResponse(
            ok=True,
            subscription=LivekitRosSubscriptionInfo(
                topic="/foo",
                type="std_msgs/msg/String",
                reliability=RosSubscriptionReliability.RELIABLE,
                depth=10,
            ),
            status=LivekitRpcSubscriptionStatus(
                applied_interval_ms=100,
                requester_count=1,
            ),
        )


def test_subscribe_response_serializes_data_transport() -> None:
    response = LivekitRpcSubscribeResponse(
        ok=True,
        subscription=LivekitRosSubscriptionInfo(
            topic="/foo",
            type="std_msgs/msg/String",
            reliability=RosSubscriptionReliability.RELIABLE,
            depth=10,
        ),
        status=LivekitRpcSubscriptionStatus(
            applied_interval_ms=100,
            requester_count=1,
        ),
        transport=LivekitRpcSubscriptionDataTransport(topic="ros.topic.messages"),
    )

    payload = response.dict()

    assert payload["transport"] == {
        "kind": "data",
        "topic": "ros.topic.messages",
    }


def test_subscribe_response_serializes_video_transport() -> None:
    response = LivekitRpcSubscribeResponse(
        ok=True,
        subscription=LivekitRosSubscriptionInfo(
            topic="/camera/image",
            type="sensor_msgs/msg/Image",
            reliability=RosSubscriptionReliability.BEST_EFFORT,
            depth=5,
        ),
        status=LivekitRpcSubscriptionStatus(
            applied_interval_ms=0,
            requester_count=1,
        ),
        transport=LivekitRpcSubscriptionVideoTransport(
            track_name="ros-video:/camera/image"
        ),
    )

    payload = response.dict()

    assert payload["transport"] == {
        "kind": "video",
        "track_name": "ros-video:/camera/image",
    }
