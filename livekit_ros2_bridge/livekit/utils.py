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

import logging
from typing import Any

from livekit import rtc

logger = logging.getLogger(__name__)

_PARTICIPANT_ID_FIELDS = ("identity", "sid", "name")


def extract_requester_id(data: Any) -> str | None:
    candidate = getattr(data, "caller_identity", None)
    if isinstance(candidate, str):
        candidate = candidate.strip()
        return candidate or None
    return None


def get_participant_id_from_packet(data: Any) -> str | None:
    participant_identity = getattr(data, "participant_identity", None)
    if participant_identity:
        return str(participant_identity)

    participant = getattr(data, "participant", None)
    if participant is not None:
        return extract_participant_id(participant)

    participant_sid = getattr(data, "participant_sid", None)
    if participant_sid:
        logger.warning(
            "LiveKit data packet missing participant_identity; using participant_sid=%s",
            participant_sid,
        )
        return str(participant_sid)

    return None


def extract_participant_id(participant: Any) -> str | None:
    if participant is None:
        return None

    for attr in _PARTICIPANT_ID_FIELDS:
        value = getattr(participant, attr, None)
        if value:
            return str(value)
    return None


def normalize_livekit_topic(topic: str | None) -> str:
    if topic is None:
        return ""
    try:
        return str(topic)
    except Exception:
        return ""


def is_room_connected(room: rtc.Room | None) -> bool:
    if room is None:
        return False
    return room.isconnected()


def decode_payload_text(raw: Any) -> str:
    if isinstance(raw, str):
        return raw
    if isinstance(raw, (bytes, bytearray)):
        return bytes(raw).decode("utf-8")
    if isinstance(raw, memoryview):
        return raw.tobytes().decode("utf-8")
    return raw.decode("utf-8")


def extract_publish_metadata(
    payload: dict[str, Any] | None,
) -> tuple[str | None, str | None]:
    if not payload:
        return None, None

    raw_topic: str | None = None
    ros_type: str | None = None

    candidate_topic = payload.get("topic")
    if isinstance(candidate_topic, str):
        raw_topic = candidate_topic

    candidate_type = payload.get("type")
    if isinstance(candidate_type, str):
        ros_type = candidate_type.strip() or None

    return raw_topic, ros_type


__all__ = [
    "decode_payload_text",
    "extract_participant_id",
    "extract_publish_metadata",
    "extract_requester_id",
    "get_participant_id_from_packet",
    "is_room_connected",
    "normalize_livekit_topic",
]
