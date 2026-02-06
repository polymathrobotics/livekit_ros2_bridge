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

from typing import Any, cast
from unittest.mock import MagicMock

from test.support.router_harness import (
    DummyLocalParticipant,
    DummySubscriber,
    make_router,
)
from livekit_ros2_bridge.core.protocol import (
    RPC_SERVICE_CALL,
    RPC_TOPIC_SUBSCRIBE,
    RPC_TOPIC_UNSUBSCRIBE,
)


def test_register_rpc_methods_reregisters_after_clear() -> None:
    subscriber = DummySubscriber()
    router = make_router(subscriber=subscriber)
    local_participant = DummyLocalParticipant()

    router.register_rpc_methods(cast(Any, local_participant))
    assert set(local_participant.handlers.keys()) == {
        RPC_TOPIC_SUBSCRIBE,
        RPC_TOPIC_UNSUBSCRIBE,
        RPC_SERVICE_CALL,
    }

    local_participant.handlers = {}
    router.clear_livekit_requesters()
    router.register_rpc_methods(cast(Any, local_participant))

    assert subscriber.cleared is True
    assert set(local_participant.handlers.keys()) == {
        RPC_TOPIC_SUBSCRIBE,
        RPC_TOPIC_UNSUBSCRIBE,
        RPC_SERVICE_CALL,
    }


def test_register_rpc_methods_is_idempotent_for_same_participant() -> None:
    router = make_router()
    local_participant = DummyLocalParticipant()

    router.register_rpc_methods(cast(Any, local_participant))
    first_count = len(local_participant.register_calls)

    router.register_rpc_methods(cast(Any, local_participant))

    assert len(local_participant.register_calls) == first_count


def test_register_rpc_methods_ignores_missing_participant() -> None:
    router = make_router()
    router.register_rpc_methods(None)


def test_register_rpc_methods_skips_service_call_when_disabled() -> None:
    router = make_router(service_caller=None)
    local_participant = DummyLocalParticipant()

    router.register_rpc_methods(cast(Any, local_participant))

    assert set(local_participant.handlers.keys()) == {
        RPC_TOPIC_SUBSCRIBE,
        RPC_TOPIC_UNSUBSCRIBE,
    }


def test_remove_participant_accepts_string() -> None:
    subscriber = DummySubscriber()
    router = make_router(subscriber=subscriber)

    router.remove_participant("participant-123")

    assert subscriber.removed_participants == ["participant-123"]


def test_remove_participant_extracts_identity_from_object() -> None:
    subscriber = DummySubscriber()
    router = make_router(subscriber=subscriber)
    participant = MagicMock()
    participant.identity = "participant-123"

    router.remove_participant(participant)

    assert subscriber.removed_participants == ["participant-123"]


def test_remove_participant_ignores_objects_without_identity() -> None:
    subscriber = DummySubscriber()
    router = make_router(subscriber=subscriber)
    participant = MagicMock()
    participant.identity = None
    participant.sid = None
    participant.name = None

    router.remove_participant(participant)

    assert subscriber.removed_participants == []


def test_shutdown_calls_subscriber_shutdown() -> None:
    subscriber = DummySubscriber()
    router = make_router(subscriber=subscriber)

    router.shutdown()

    assert subscriber.shutdown_calls == 1
