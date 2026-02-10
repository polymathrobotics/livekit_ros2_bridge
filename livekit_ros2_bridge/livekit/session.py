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
import datetime
import threading
from dataclasses import dataclass
from typing import Any, Protocol

import livekit.api as api
import livekit.rtc as rtc

from livekit_ros2_bridge.livekit.utils import is_room_connected


@dataclass(frozen=True)
class LivekitConnectConfig:
    url: str
    room: str
    identity: str


class TokenSource(Protocol):
    def get_token(self, room: str, identity: str) -> str: ...


@dataclass(frozen=True)
class StaticTokenSource:
    token: str

    def get_token(self, room: str, identity: str) -> str:
        return self.token


@dataclass(frozen=True)
class ApiKeyTokenSource:
    api_key: str
    api_secret: str
    ttl_seconds: int = 3600

    def get_token(self, room: str, identity: str) -> str:
        return (
            api.AccessToken(api_key=self.api_key, api_secret=self.api_secret)
            .with_identity(identity)
            .with_ttl(datetime.timedelta(seconds=self.ttl_seconds))
            .with_grants(
                api.VideoGrants(
                    room_join=True,
                    room=room,
                    # The bridge uses data packets + RPC, not media tracks.
                    can_subscribe=True,
                    can_publish_data=True,
                )
            )
            .to_jwt()
        )


class RoomEventHandler(Protocol):
    def on_connected(self, local_participant: rtc.LocalParticipant | None) -> None: ...

    def on_data_received(self, data: rtc.DataPacket) -> None: ...

    def on_participant_disconnected(
        self, participant: rtc.RemoteParticipant
    ) -> None: ...

    def on_session_reset(self) -> None: ...


class LivekitSession:
    def __init__(
        self,
        *,
        config: LivekitConnectConfig,
        token_source: TokenSource,
        handler: RoomEventHandler,
        logger: Any,
        initial_backoff_ms: int,
        max_backoff_ms: int,
    ) -> None:
        self._config = config
        self._token_source = token_source
        self._handler = handler
        self._logger = logger

        self._initial_backoff_s = max(initial_backoff_ms, 0) / 1000.0
        self._max_backoff_s = max(max_backoff_ms, 0) / 1000.0
        if self._max_backoff_s < self._initial_backoff_s:
            self._max_backoff_s = self._initial_backoff_s

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread_started = False
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop_async: asyncio.Event | None = None
        self._room: rtc.Room | None = None

    def get_room(self) -> rtc.Room | None:
        return self._room

    def get_loop(self) -> asyncio.AbstractEventLoop | None:
        return self._loop

    def start(self) -> None:
        if self._thread_started:
            return
        self._thread_started = True
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        loop = self._loop
        if loop and loop.is_running():
            loop.call_soon_threadsafe(self._signal_stop_async)
        if self._thread.is_alive():
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                self._logger.warning(
                    "Timed out waiting for LiveKit session thread to stop."
                )

    def _thread_main(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._stop_async = asyncio.Event()
        if self._stop_event.is_set():
            self._stop_async.set()
        try:
            self._loop.run_until_complete(self._run())
        finally:
            self._loop.close()

    async def _run(self) -> None:
        backoff = self._initial_backoff_s
        while not self._stop_event.is_set():
            try:
                await self._connect_once()
                backoff = self._initial_backoff_s
                await self._monitor_connection()
            except asyncio.CancelledError:
                break
            except Exception as exc:
                self._logger.error(f"LiveKit session error: {exc}")
            finally:
                await self._disconnect()
                try:
                    self._handler.on_session_reset()
                except Exception as exc:
                    self._logger.error(f"LiveKit session reset handler failed: {exc}")

            if self._stop_event.is_set():
                break
            if backoff:
                self._logger.warning(f"Reconnecting to LiveKit in {backoff:.1f}s")
                await self._sleep_until_stop(backoff)
                backoff = min(backoff * 2, self._max_backoff_s)

    async def _connect_once(self) -> None:
        token = self._token_source.get_token(self._config.room, self._config.identity)
        self._room = rtc.Room()
        self._room.on("data_received", self._handle_data_received)
        self._room.on("participant_disconnected", self._handle_participant_disconnected)

        await self._room.connect(self._config.url, token)
        self._logger.info(
            f"Connected to LiveKit room={self._config.room} identity={self._config.identity}"
        )
        local_participant: rtc.LocalParticipant | None = None
        try:
            local_participant = self._room.local_participant
        except Exception:
            local_participant = None
        self._handler.on_connected(local_participant)

    async def _monitor_connection(self) -> None:
        while not self._stop_event.is_set():
            if not is_room_connected(self._room):
                break
            await self._sleep_until_stop(1.0)

    async def _disconnect(self) -> None:
        room = self._room
        self._room = None
        if room is None:
            return
        if is_room_connected(room):
            try:
                await room.disconnect()
                self._logger.info("Disconnected from LiveKit")
            except Exception as exc:
                self._logger.error(f"Failed to disconnect from LiveKit: {exc}")

    def _signal_stop_async(self) -> None:
        stop_async = self._stop_async
        if stop_async is not None:
            stop_async.set()

    async def _sleep_until_stop(self, timeout_s: float) -> None:
        if timeout_s <= 0:
            await asyncio.sleep(0)
            return

        stop_async = self._stop_async
        if stop_async is None:
            await asyncio.sleep(timeout_s)
            return

        try:
            await asyncio.wait_for(stop_async.wait(), timeout=timeout_s)
        except asyncio.TimeoutError:
            return

    def _handle_data_received(self, data: rtc.DataPacket) -> None:
        try:
            self._handler.on_data_received(data)
        except Exception as exc:
            self._logger.error(f"LiveKit data packet handler failed: {exc}")

    def _handle_participant_disconnected(
        self, participant: rtc.RemoteParticipant
    ) -> None:
        try:
            self._handler.on_participant_disconnected(participant)
        except Exception as exc:
            self._logger.error(f"LiveKit participant handler failed: {exc}")


__all__ = [
    "ApiKeyTokenSource",
    "LivekitConnectConfig",
    "LivekitSession",
    "RoomEventHandler",
    "StaticTokenSource",
    "TokenSource",
]
