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
from typing import Any

from livekit_ros2_bridge.core.access_static import StaticAccessPolicy
from livekit_ros2_bridge.core.telemetry import NullTelemetry
from livekit_ros2_bridge.livekit.session import LivekitConnectConfig, StaticTokenSource
from livekit_ros2_bridge.ros2.publisher import PublisherConfig
from livekit_ros2_bridge.ros2.service_caller import ServiceConfig
from livekit_ros2_bridge.ros2.subscription_registry import SubscriberConfig
from livekit_ros2_bridge.runtime import Runtime, RuntimeConfig


class DummyLogger:
    def __init__(self) -> None:
        self.messages: list[str] = []

    def warning(self, message: str) -> None:
        self.messages.append(message)

    def info(self, message: str) -> None:
        self.messages.append(message)

    def error(self, message: str) -> None:
        self.messages.append(message)


class DummyNode:
    def __init__(self) -> None:
        self.logger = DummyLogger()

    def get_logger(self) -> DummyLogger:
        return self.logger


class DummyDispatcher:
    def __init__(self, node: Any, *, callback_group: Any) -> None:
        del node, callback_group
        self.shutdown_calls = 0

    def shutdown(self) -> None:
        self.shutdown_calls += 1


class DummySession:
    def __init__(self, **kwargs: Any) -> None:
        self.kwargs = kwargs
        self.started = 0
        self.stopped = 0

    def start(self) -> None:
        self.started += 1

    def stop(self) -> None:
        self.stopped += 1

    def get_room(self) -> None:
        return None

    def get_loop(self) -> None:
        return None


class DummyServiceCaller:
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        del args, kwargs
        self.shutdown_calls = 0

    def shutdown(self) -> None:
        self.shutdown_calls += 1


class DummyRouter:
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        del args, kwargs
        self.registered: list[Any] = []
        self.received: list[Any] = []
        self.removed: list[Any] = []
        self.cleared = 0
        self.shutdown_calls = 0

    def register_rpc_methods(self, local_participant: Any) -> None:
        self.registered.append(local_participant)

    def handle_data_packet(self, data: Any) -> None:
        self.received.append(data)

    def remove_participant(self, participant: Any) -> None:
        self.removed.append(participant)

    def clear_livekit_requesters(self) -> None:
        self.cleared += 1

    def shutdown(self) -> None:
        self.shutdown_calls += 1


@dataclass
class RuntimeObjects:
    dispatcher: DummyDispatcher
    session: DummySession
    service_caller: DummyServiceCaller
    router: DummyRouter


def _make_runtime(monkeypatch: Any) -> tuple[Runtime, RuntimeObjects]:
    holder: dict[str, Any] = {}

    monkeypatch.setattr(
        "livekit_ros2_bridge.runtime.MutuallyExclusiveCallbackGroup", object
    )

    def _dispatcher(node: Any, *, callback_group: Any) -> DummyDispatcher:
        instance = DummyDispatcher(node, callback_group=callback_group)
        holder["dispatcher"] = instance
        return instance

    def _session(**kwargs: Any) -> DummySession:
        instance = DummySession(**kwargs)
        holder["session"] = instance
        return instance

    def _service_caller(*args: Any, **kwargs: Any) -> DummyServiceCaller:
        instance = DummyServiceCaller(*args, **kwargs)
        holder["service_caller"] = instance
        return instance

    def _router(*args: Any, **kwargs: Any) -> DummyRouter:
        instance = DummyRouter(*args, **kwargs)
        holder["router"] = instance
        return instance

    monkeypatch.setattr(
        "livekit_ros2_bridge.runtime.RosExecutorDispatcher", _dispatcher
    )
    monkeypatch.setattr("livekit_ros2_bridge.runtime.LivekitSession", _session)
    monkeypatch.setattr(
        "livekit_ros2_bridge.runtime.LivekitRoomPublisher", lambda **kwargs: object()
    )
    monkeypatch.setattr(
        "livekit_ros2_bridge.runtime.RosSubscriptionRegistry",
        lambda *args, **kwargs: object(),
    )
    monkeypatch.setattr("livekit_ros2_bridge.runtime.RosServiceCaller", _service_caller)
    monkeypatch.setattr(
        "livekit_ros2_bridge.runtime.RosPublisherRegistry",
        lambda *args, **kwargs: object(),
    )
    monkeypatch.setattr(
        "livekit_ros2_bridge.runtime.RosPublisher", lambda *args, **kwargs: object()
    )
    monkeypatch.setattr("livekit_ros2_bridge.runtime.LivekitRouter", _router)

    runtime = Runtime(
        DummyNode(),
        connect_config=LivekitConnectConfig(
            url="wss://example.test",
            room="room",
            identity="test-identity",
        ),
        token_source=StaticTokenSource(token="token"),
        runtime_config=RuntimeConfig(
            subscriber=SubscriberConfig(),
            publisher=PublisherConfig(),
            service=ServiceConfig(),
        ),
        access_policy=StaticAccessPolicy(subscribe_allow=["*"]),
        telemetry=NullTelemetry(),
    )
    objects = RuntimeObjects(
        dispatcher=holder["dispatcher"],
        session=holder["session"],
        service_caller=holder["service_caller"],
        router=holder["router"],
    )
    return runtime, objects


def test_runtime_forwards_events_until_shutdown(monkeypatch: Any) -> None:
    runtime, objects = _make_runtime(monkeypatch)

    runtime.start()
    assert objects.session.started == 1

    local_participant = object()
    packet = object()
    participant = object()

    runtime.on_connected(local_participant)
    runtime.on_data_received(packet)
    runtime.on_participant_disconnected(participant)
    runtime.on_session_reset()

    assert objects.router.registered == [local_participant]
    assert objects.router.received == [packet]
    assert objects.router.removed == [participant]
    assert objects.router.cleared == 1


def test_runtime_shutdown_stops_forwarding_and_shuts_down_components(
    monkeypatch: Any,
) -> None:
    runtime, objects = _make_runtime(monkeypatch)

    runtime.shutdown()

    assert objects.service_caller.shutdown_calls == 1
    assert objects.session.stopped == 1
    assert objects.router.shutdown_calls == 1
    assert objects.dispatcher.shutdown_calls == 1

    runtime.on_connected(object())
    runtime.on_data_received(object())
    runtime.on_participant_disconnected(object())
    runtime.on_session_reset()

    assert objects.router.registered == []
    assert objects.router.received == []
    assert objects.router.removed == []
    assert objects.router.cleared == 0
