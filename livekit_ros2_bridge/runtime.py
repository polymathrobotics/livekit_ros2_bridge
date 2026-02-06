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

from dataclasses import dataclass
from typing import Iterable

from livekit import rtc
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from livekit_ros2_bridge.core.access import AccessPolicy
from livekit_ros2_bridge.core.telemetry import Telemetry
from livekit_ros2_bridge.livekit.router import LivekitRouter
from livekit_ros2_bridge.livekit.room_publisher import LivekitRoomPublisher
from livekit_ros2_bridge.livekit.session import (
    LivekitConnectConfig,
    LivekitSession,
    RoomEventHandler,
    TokenSource,
)
from livekit_ros2_bridge.ros2.executor_dispatcher import RosExecutorDispatcher
from livekit_ros2_bridge.ros2.publisher import PublisherConfig, RosPublisher
from livekit_ros2_bridge.ros2.service_caller import (
    RosServiceCaller,
    ServiceConfig,
)
from livekit_ros2_bridge.ros2.publisher_registry import RosPublisherRegistry
from livekit_ros2_bridge.ros2.subscription_registry import (
    RosSubscriptionRegistry,
    SubscriberConfig,
)


@dataclass(frozen=True)
class AccessPolicyConfig:
    publish_allow: Iterable[str] = ()
    publish_deny: Iterable[str] = ()
    subscribe_allow: Iterable[str] = ()
    subscribe_deny: Iterable[str] = ()
    subscribe_type_deny: Iterable[str] = ()
    service_allow: Iterable[str] = ()
    service_deny: Iterable[str] = ()


@dataclass(frozen=True)
class RuntimeConfig:
    subscriber: SubscriberConfig
    publisher: PublisherConfig
    service: ServiceConfig
    initial_backoff_ms: int = 500
    max_backoff_ms: int = 10000


class Runtime(RoomEventHandler):
    def __init__(
        self,
        node: Node,
        *,
        connect_config: LivekitConnectConfig,
        token_source: TokenSource,
        runtime_config: RuntimeConfig,
        access_policy: AccessPolicy,
        telemetry: Telemetry,
    ) -> None:
        self._shutting_down = False
        callback_group = MutuallyExclusiveCallbackGroup()
        self._ros_dispatcher = RosExecutorDispatcher(
            node,
            callback_group=callback_group,
        )
        self._session = self._create_session(
            node=node,
            connect_config=connect_config,
            token_source=token_source,
            runtime_config=runtime_config,
        )
        room_publisher = self._create_room_publisher()

        ros_subscription_registry = RosSubscriptionRegistry(
            node,
            room_publisher,
            runtime_config.subscriber,
            access_policy=access_policy,
            telemetry=telemetry,
            callback_group=callback_group,
        )
        self._service_caller = RosServiceCaller(
            node,
            runtime_config.service,
            access_policy=access_policy,
            telemetry=telemetry,
            callback_group=callback_group,
        )
        ros_publisher_registry = RosPublisherRegistry(
            node, callback_group=callback_group
        )
        ros_publisher = RosPublisher(
            ros_publisher_registry,
            runtime_config.publisher,
            access_policy=access_policy,
            telemetry=telemetry,
        )

        self._router = LivekitRouter(
            ros_publisher,
            ros_subscription_registry,
            service_caller=self._service_caller,
            work_dispatcher=self._ros_dispatcher,
            telemetry=telemetry,
            service_default_timeout_ms=runtime_config.service.default_timeout_ms,
        )

    def _create_session(
        self,
        *,
        node: Node,
        connect_config: LivekitConnectConfig,
        token_source: TokenSource,
        runtime_config: RuntimeConfig,
    ) -> LivekitSession:
        return LivekitSession(
            config=connect_config,
            token_source=token_source,
            handler=self,
            logger=node.get_logger(),
            initial_backoff_ms=runtime_config.initial_backoff_ms,
            max_backoff_ms=runtime_config.max_backoff_ms,
        )

    def _create_room_publisher(self) -> LivekitRoomPublisher:
        return LivekitRoomPublisher(
            room_provider=self._session.get_room,
            loop_provider=self._session.get_loop,
        )

    def start(self) -> None:
        self._session.start()

    def shutdown(self) -> None:
        self._shutting_down = True
        self._service_caller.shutdown()
        self._session.stop()
        self._router.shutdown()
        self._ros_dispatcher.shutdown()

    def on_connected(self, local_participant: rtc.LocalParticipant | None) -> None:
        if self._shutting_down:
            return
        self._router.register_rpc_methods(local_participant)

    def on_data_received(self, data: rtc.DataPacket) -> None:
        if self._shutting_down:
            return
        self._router.handle_data_packet(data)

    def on_participant_disconnected(self, participant: rtc.RemoteParticipant) -> None:
        if self._shutting_down:
            return
        self._router.remove_participant(participant)

    def on_session_reset(self) -> None:
        if self._shutting_down:
            return
        self._router.clear_livekit_requesters()


__all__ = [
    "AccessPolicyConfig",
    "Runtime",
    "RuntimeConfig",
]
