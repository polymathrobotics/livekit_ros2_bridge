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

from types import SimpleNamespace
from typing import Any, cast

from livekit_ros2_bridge.livekit.utils import (
    decode_payload_text,
    extract_participant_id,
    extract_publish_metadata,
    extract_requester_id,
    get_participant_id_from_packet,
    is_room_connected,
    normalize_livekit_topic,
)


def test_extract_requester_id_trims_whitespace() -> None:
    data = SimpleNamespace(caller_identity="  participant-1  ")

    assert extract_requester_id(data) == "participant-1"
    assert extract_requester_id(SimpleNamespace(caller_identity="   ")) is None
    assert extract_requester_id(SimpleNamespace(caller_identity=None)) is None


def test_get_participant_id_from_packet_prefers_identity() -> None:
    packet = SimpleNamespace(participant_identity="participant-123")

    assert get_participant_id_from_packet(packet) == "participant-123"


def test_get_participant_id_from_packet_falls_back_to_participant_fields() -> None:
    participant = SimpleNamespace(identity=None, sid="P-SID", name="P-NAME")
    packet = SimpleNamespace(participant_identity=None, participant=participant)

    assert get_participant_id_from_packet(packet) == "P-SID"


def test_get_participant_id_from_packet_falls_back_to_sid() -> None:
    packet = SimpleNamespace(
        participant_identity=None,
        participant=None,
        participant_sid="PSID-1",
    )

    assert get_participant_id_from_packet(packet) == "PSID-1"


def test_extract_participant_id_checks_identity_sid_name() -> None:
    assert (
        extract_participant_id(SimpleNamespace(identity="id", sid="sid", name="n"))
        == "id"
    )
    assert (
        extract_participant_id(SimpleNamespace(identity=None, sid="sid", name="n"))
        == "sid"
    )
    assert (
        extract_participant_id(SimpleNamespace(identity=None, sid=None, name="n"))
        == "n"
    )
    assert extract_participant_id(None) is None


def test_normalize_livekit_topic_handles_none_and_bad_values() -> None:
    class _BadToStr:
        def __str__(self) -> str:
            raise RuntimeError("bad")

    assert normalize_livekit_topic(None) == ""
    assert normalize_livekit_topic("/foo") == "/foo"
    assert normalize_livekit_topic(cast(Any, _BadToStr())) == ""


def test_is_room_connected() -> None:
    class _Room:
        def __init__(self, connected: bool) -> None:
            self._connected = connected

        def isconnected(self) -> bool:
            return self._connected

    assert is_room_connected(None) is False
    assert is_room_connected(cast(Any, _Room(True))) is True
    assert is_room_connected(cast(Any, _Room(False))) is False


def test_decode_payload_text_supports_common_buffer_types() -> None:
    assert decode_payload_text("hello") == "hello"
    assert decode_payload_text(b"hello") == "hello"
    assert decode_payload_text(bytearray(b"hello")) == "hello"
    assert decode_payload_text(memoryview(b"hello")) == "hello"


def test_extract_publish_metadata() -> None:
    payload: dict[str, Any] = {"topic": "/foo", "type": " std_msgs/msg/String "}
    assert extract_publish_metadata(payload) == ("/foo", "std_msgs/msg/String")
    assert extract_publish_metadata({"topic": 1, "type": None}) == (None, None)
    assert extract_publish_metadata(None) == (None, None)
