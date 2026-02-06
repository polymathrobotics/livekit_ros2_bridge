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
import logging
from collections.abc import Callable, Coroutine
from typing import Any, Mapping

from livekit import rtc

from livekit_ros2_bridge.core.room_publisher import RoomPublisher
from livekit_ros2_bridge.core.serialization import sanitize_payload
from livekit_ros2_bridge.livekit.utils import is_room_connected

logger = logging.getLogger(__name__)

LIVEKIT_RELIABLE_PACKET_WARN_BYTES = 15 * 1024


class LivekitRoomPublisher(RoomPublisher):
    """RoomPublisher implementation that publishes over a LiveKit room."""

    def __init__(
        self,
        *,
        room_provider: Callable[[], rtc.Room | None],
        loop_provider: Callable[[], asyncio.AbstractEventLoop | None],
    ) -> None:
        self._room_provider = room_provider
        self._loop_provider = loop_provider

    def publish_data(
        self,
        topic: str,
        payload: Mapping[str, Any],
        reliable: bool = True,
        *,
        destination_identities: list[str] | None = None,
    ) -> None:
        room = self._room_provider()
        if not is_room_connected(room):
            logger.info("Cannot publish data: not connected to a room.")
            return

        try:
            sanitized_payload = sanitize_payload(payload)
            message = json.dumps(sanitized_payload, allow_nan=False)
            message_bytes = message.encode("utf-8")
        except Exception as e:
            logger.error(
                "Failed to serialize LiveKit data payload: %s", e, exc_info=True
            )
            return

        message_size = len(message_bytes)
        if message_size > LIVEKIT_RELIABLE_PACKET_WARN_BYTES:
            logger.warning(
                "LiveKit payload size %s bytes exceeds recommended %s bytes for topic=%s reliable=%s",
                message_size,
                LIVEKIT_RELIABLE_PACKET_WARN_BYTES,
                topic,
                reliable,
            )

        target_identities = (
            list(destination_identities) if destination_identities else []
        )

        async def _publish_async() -> None:
            active_room = self._room_provider()
            if not is_room_connected(active_room):
                return
            if active_room is None:
                return

            try:
                local_participant = active_room.local_participant
            except Exception as e:
                logger.warning(
                    "Cannot publish data: local participant unavailable: %s", e
                )
                return

            try:
                await local_participant.publish_data(
                    message,
                    reliable=reliable,
                    destination_identities=target_identities,
                    topic=topic,
                )
            except Exception as e:
                logger.error(
                    "Failed to publish LiveKit data topic=%s: %s",
                    topic,
                    e,
                    exc_info=True,
                )

        self._schedule_coroutine(
            _publish_async,
            no_loop_message="Cannot publish data: no running event loop available.",
        )

    def can_publish_data(self) -> bool:
        room = self._room_provider()
        if not is_room_connected(room):
            return False
        if room is None:
            return False
        return self._has_remote_participants(room)

    def _has_remote_participants(self, room: rtc.Room) -> bool:
        return bool(room.remote_participants)

    def _schedule_coroutine(
        self,
        coroutine_factory: Callable[[], Coroutine[Any, Any, None]],
        *,
        no_loop_message: str,
    ) -> None:
        running_loop = None
        try:
            running_loop = asyncio.get_running_loop()
        except RuntimeError:
            pass

        loop = self._loop_provider()
        if loop and loop.is_running():
            if running_loop is loop:
                asyncio.create_task(coroutine_factory())
            else:
                asyncio.run_coroutine_threadsafe(coroutine_factory(), loop)
            return

        if running_loop:
            asyncio.create_task(coroutine_factory())
            return

        logger.warning(no_loop_message)


__all__ = ["LivekitRoomPublisher"]
