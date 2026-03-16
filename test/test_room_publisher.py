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

import asyncio
import json
from typing import Any

import pytest

from livekit_ros2_bridge.livekit.room_publisher import LivekitRoomPublisher
from livekit_ros2_bridge.ros2.ros_logging import RosLogger
from test.support.logger_harness import DummyLogger


class DummyLocalParticipant:
    def __init__(self) -> None:
        self.calls: list[dict[str, Any]] = []

    async def publish_data(
        self,
        payload: str,
        *,
        reliable: bool = True,
        destination_identities: list[str] | None = None,
        topic: str = "",
    ) -> None:
        self.calls.append(
            {
                "payload": payload,
                "reliable": reliable,
                "destination_identities": destination_identities,
                "topic": topic,
            }
        )


class DummyRoom:
    def __init__(
        self, *, connected: bool, remote_participants: dict[str, object]
    ) -> None:
        self._connected = connected
        self.remote_participants = remote_participants
        self.local_participant: Any = DummyLocalParticipant()

    def isconnected(self) -> bool:
        return self._connected


@pytest.mark.asyncio
async def test_publish_data_sends_payload_when_room_connected() -> None:
    room = DummyRoom(connected=True, remote_participants={"p1": object()})
    publisher = LivekitRoomPublisher(
        room_provider=lambda: room,
        loop_provider=lambda: asyncio.get_running_loop(),
        logger=RosLogger(DummyLogger("livekit_bridge.livekit.room_publisher")),
    )

    publisher.publish_data(
        "ros.topic.messages",
        {"msg": {"data": b"ab"}},
        destination_identities=["participant-1"],
    )
    await asyncio.sleep(0)

    calls = room.local_participant.calls
    assert len(calls) == 1
    assert calls[0]["topic"] == "ros.topic.messages"
    assert calls[0]["destination_identities"] == ["participant-1"]
    payload = json.loads(calls[0]["payload"])
    assert payload["msg"]["data"] == [97, 98]


def test_publish_data_is_noop_when_disconnected() -> None:
    room = DummyRoom(connected=False, remote_participants={})
    backend = DummyLogger("livekit_bridge.livekit.room_publisher")
    publisher = LivekitRoomPublisher(
        room_provider=lambda: room,
        loop_provider=lambda: None,
        logger=RosLogger(backend),
    )

    publisher.publish_data("ros.topic.messages", {"msg": {"data": "hello"}})

    assert room.local_participant.calls == []
    assert len(backend.records) == 1
    assert backend.records[0].level == "debug"
    assert backend.records[0].message == "Cannot publish data: not connected to a room."
    assert backend.records[0].kwargs == {"throttle_duration_sec": 30.0}


def test_publish_data_handles_unserializable_payload() -> None:
    room = DummyRoom(connected=True, remote_participants={"p1": object()})
    publisher = LivekitRoomPublisher(
        room_provider=lambda: room,
        loop_provider=lambda: None,
        logger=RosLogger(DummyLogger("livekit_bridge.livekit.room_publisher")),
    )

    # Object() is not JSON serializable.
    publisher.publish_data("ros.topic.messages", {"msg": object()})

    assert room.local_participant.calls == []


def test_publish_data_logs_missing_loop_with_throttle() -> None:
    room = DummyRoom(connected=True, remote_participants={"p1": object()})
    backend = DummyLogger("livekit_bridge.livekit.room_publisher")
    publisher = LivekitRoomPublisher(
        room_provider=lambda: room,
        loop_provider=lambda: None,
        logger=RosLogger(backend),
    )

    publisher.publish_data("ros.topic.messages", {"msg": {"data": "hello"}})

    assert room.local_participant.calls == []
    assert len(backend.records) == 1
    assert backend.records[0].level == "warning"
    assert backend.records[0].message == (
        "Cannot publish data: no running event loop available."
    )
    assert backend.records[0].kwargs == {"throttle_duration_sec": 30.0}


def test_can_publish_data_checks_connection_and_remote_participants() -> None:
    disconnected = DummyRoom(connected=False, remote_participants={"p1": object()})
    connected_empty = DummyRoom(connected=True, remote_participants={})
    connected_with_remote = DummyRoom(
        connected=True, remote_participants={"p1": object()}
    )

    assert (
        LivekitRoomPublisher(
            room_provider=lambda: disconnected,
            loop_provider=lambda: None,
            logger=RosLogger(DummyLogger("livekit_bridge.livekit.room_publisher")),
        ).can_publish_data()
        is False
    )
    assert (
        LivekitRoomPublisher(
            room_provider=lambda: connected_empty,
            loop_provider=lambda: None,
            logger=RosLogger(DummyLogger("livekit_bridge.livekit.room_publisher")),
        ).can_publish_data()
        is False
    )
    assert (
        LivekitRoomPublisher(
            room_provider=lambda: connected_with_remote,
            loop_provider=lambda: None,
            logger=RosLogger(DummyLogger("livekit_bridge.livekit.room_publisher")),
        ).can_publish_data()
        is True
    )
