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

from enum import Enum
from typing import Any

from pydantic import BaseModel, Field

DATA_TOPIC = "ros.topic.messages"
PUBLISH_TOPIC = "ros.topic.publish"
RPC_TOPIC_SUBSCRIBE = "ros.topic.subscribe"
RPC_TOPIC_UNSUBSCRIBE = "ros.topic.unsubscribe"
RPC_SERVICE_CALL = "ros.service.call"

# LiveKit RPC error codes (stable client contract; see docs/protocol.md).
#
# Codes intentionally mirror HTTP status codes but stay in the 2xxx range to
# avoid collisions with other LiveKit/client error domains.
RPC_ERROR_INVALID_REQUEST = 2400
RPC_ERROR_UNAUTHORIZED = 2401
RPC_ERROR_FORBIDDEN = 2403
RPC_ERROR_INTERNAL = 2500

PROTOCOL_VERSION = 1
DATA_CONTENT_TYPE = "application/json"


class RosSubscriptionReliability(str, Enum):
    """Reliability modes that LiveKit clients can request for ROS subscriptions."""

    RELIABLE = "reliable"
    BEST_EFFORT = "best_effort"


class LivekitRosSubscriptionInfo(BaseModel):
    """Resolved configuration for a ROS topic subscription."""

    depth: int
    reliability: RosSubscriptionReliability
    topic: str
    type: str


class LivekitRosMessageBody(BaseModel):
    """Body of a LiveKit data-channel message carrying a ROS topic payload."""

    content_type: str
    msg: Any
    type: str
    topic: str


class LivekitRosMessageEnvelope(BaseModel):
    """Envelope for ROS bridge messages sent on the LiveKit data channel."""

    v: int
    type: str
    id: str
    body: LivekitRosMessageBody


class LivekitRpcRequestBase(BaseModel):
    """Base model for LiveKit RPC requests handled by `LivekitRouter`."""

    class Config:
        # Ignore unknown fields so newer protocol versions still parse on older robots.
        extra = "ignore"
        anystr_strip_whitespace = True


class LivekitRpcSubscribeRequest(LivekitRpcRequestBase):
    """Payload for the `ros.topic.subscribe` RPC that requests a ROS topic subscription."""

    topic: str = Field(min_length=1)
    preferred_interval_ms: int = Field(default=0, ge=0)


class LivekitRpcUnsubscribeRequest(LivekitRpcRequestBase):
    """Payload for the `ros.topic.unsubscribe` RPC that tears down a ROS topic subscription."""

    topic: str = Field(min_length=1)


class LivekitRpcCallServiceRequest(LivekitRpcRequestBase):
    """Payload for the `ros.service.call` RPC executed by `RosServiceCaller`."""

    service: str = Field(min_length=1)
    type: str | None = Field(default=None, min_length=1)
    request: dict[str, Any]
    timeout_ms: int | None = None


class LivekitRpcSubscriptionStatus(BaseModel):
    """Status summary returned from subscribe/unsubscribe RPCs."""

    applied_interval_ms: int
    requester_count: int


class LivekitRpcServiceResponse(BaseModel):
    """Service identity details returned from `ros.service.call` RPCs."""

    name: str
    type: str


class LivekitRpcSubscribeResponse(BaseModel):
    """Response payload for `ros.topic.subscribe` and `ros.topic.unsubscribe` RPCs."""

    ok: bool
    subscription: LivekitRosSubscriptionInfo
    status: LivekitRpcSubscriptionStatus


class LivekitRpcCallServiceResponse(BaseModel):
    """Response payload for the `ros.service.call` RPC."""

    ok: bool
    service: LivekitRpcServiceResponse
    response: dict[str, Any]
    elapsed_ms: int


class LivekitRosPublishPayload(BaseModel):
    """Payload for `ros.topic.publish` data-channel messages."""

    topic: str = Field(min_length=1)
    type: str = Field(min_length=1)
    msg: dict[str, Any]

    class Config:
        extra = "ignore"
        anystr_strip_whitespace = True


__all__ = [
    "DATA_CONTENT_TYPE",
    "DATA_TOPIC",
    "LivekitRosMessageBody",
    "LivekitRosMessageEnvelope",
    "LivekitRosPublishPayload",
    "LivekitRosSubscriptionInfo",
    "RosSubscriptionReliability",
    "LivekitRpcCallServiceRequest",
    "LivekitRpcCallServiceResponse",
    "LivekitRpcRequestBase",
    "LivekitRpcServiceResponse",
    "LivekitRpcSubscribeRequest",
    "LivekitRpcSubscribeResponse",
    "LivekitRpcSubscriptionStatus",
    "LivekitRpcUnsubscribeRequest",
    "PROTOCOL_VERSION",
    "PUBLISH_TOPIC",
    "RPC_SERVICE_CALL",
    "RPC_TOPIC_SUBSCRIBE",
    "RPC_TOPIC_UNSUBSCRIBE",
    "RPC_ERROR_INTERNAL",
    "RPC_ERROR_INVALID_REQUEST",
    "RPC_ERROR_UNAUTHORIZED",
    "RPC_ERROR_FORBIDDEN",
]
