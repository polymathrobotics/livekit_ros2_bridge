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

import socket
from typing import Any, Iterable, cast

from rclpy.node import Node

from livekit_ros2_bridge.parameters import livekit_bridge
from livekit_ros2_bridge.core.access import AccessPolicy
from livekit_ros2_bridge.core.access_static import StaticAccessPolicy
from livekit_ros2_bridge.core.extensions import load_symbol
from livekit_ros2_bridge.core.telemetry import NullTelemetry, Telemetry
from livekit_ros2_bridge.livekit.session import (
    ApiKeyTokenSource,
    LivekitConnectConfig,
    StaticTokenSource,
    TokenSource,
)
from livekit_ros2_bridge.ros2.publisher import PublisherConfig
from livekit_ros2_bridge.ros2.service_caller import ServiceConfig
from livekit_ros2_bridge.ros2.subscription_registry import SubscriberConfig
from livekit_ros2_bridge.runtime import (
    AccessPolicyConfig,
    Runtime,
    RuntimeConfig,
)

_REQUIRED_ACCESS_POLICY_METHODS: tuple[str, ...] = ("authorize",)
_REQUIRED_TELEMETRY_METHODS: tuple[str, ...] = ("emit",)
_ACCESS_FACTORY_PARAMETER = "access.factory"
_TELEMETRY_FACTORY_PARAMETER = "telemetry.factory"


class LivekitBridgeNode(Node):
    """ROS2 node that bridges ROS topics/services over LiveKit."""

    def __init__(self) -> None:
        super().__init__("livekit_bridge")
        self._runtime: Runtime | None = None
        param_listener = livekit_bridge.ParamListener(self)
        self._params: livekit_bridge.Params = param_listener.get_params()

        connect_config, token_source = self._load_livekit_config()
        access_config = self._load_access_config()
        runtime_config = self._load_runtime_config()

        access_policy = self._build_access_policy(access_config)
        telemetry = self._build_telemetry(runtime_config)

        self._runtime = Runtime(
            self,
            connect_config=connect_config,
            token_source=token_source,
            runtime_config=runtime_config,
            access_policy=access_policy,
            telemetry=telemetry,
        )
        self._runtime.start()

    def shutdown(self) -> None:
        runtime = self._runtime
        if runtime is not None:
            runtime.shutdown()

    def _build_access_policy(self, config: AccessPolicyConfig) -> AccessPolicy:
        factory_path = self._params.access.factory
        if factory_path:
            return self._load_access_factory(factory_path, config)

        access_policy = StaticAccessPolicy(
            publish_allow=config.publish_allow,
            publish_deny=config.publish_deny,
            subscribe_allow=config.subscribe_allow,
            subscribe_deny=config.subscribe_deny,
            subscribe_type_deny=config.subscribe_type_deny,
            service_allow=config.service_allow,
            service_deny=config.service_deny,
        )
        self._ensure_static_access_policy_configured(access_policy)
        return access_policy

    def _build_telemetry(self, config: RuntimeConfig) -> Telemetry:
        factory_path = self._params.telemetry.factory
        if not factory_path:
            return NullTelemetry()
        return self._load_telemetry_factory(factory_path, config)

    def _load_livekit_config(self) -> tuple[LivekitConnectConfig, TokenSource]:
        params = self._params
        if not params.livekit.url:
            raise ValueError("livekit.url is required")
        if not params.livekit.room:
            raise ValueError("livekit.room is required")

        identity = params.livekit.identity
        if not identity:
            identity = f"{self.get_name()}-{socket.gethostname()}"

        if params.livekit.token:
            token_source: TokenSource = StaticTokenSource(token=params.livekit.token)
            token_source_name = "static token"
        elif params.livekit.api_key and params.livekit.api_secret:
            token_source = ApiKeyTokenSource(
                api_key=params.livekit.api_key,
                api_secret=params.livekit.api_secret,
                ttl_seconds=params.livekit.token_ttl_seconds,
            )
            token_source_name = "api key/secret"
        else:
            raise ValueError(
                "Either livekit.token or livekit.api_key + livekit.api_secret must be set"
            )

        self.get_logger().info(
            f"LiveKit config loaded: url={params.livekit.url} "
            f"room={params.livekit.room} identity={identity} "
            f"token_source={token_source_name} "
            f"ttl_seconds={params.livekit.token_ttl_seconds}"
        )

        return (
            LivekitConnectConfig(
                url=params.livekit.url,
                room=params.livekit.room,
                identity=identity,
            ),
            token_source,
        )

    def _load_access_config(self) -> AccessPolicyConfig:
        access = self._params.access.static
        return AccessPolicyConfig(
            publish_allow=self._normalize_access_list_param(access.publish.allow),
            publish_deny=self._normalize_access_list_param(access.publish.deny),
            subscribe_allow=self._normalize_access_list_param(access.subscribe.allow),
            subscribe_deny=self._normalize_access_list_param(access.subscribe.deny),
            subscribe_type_deny=self._normalize_access_list_param(
                access.subscribe.type_deny
            ),
            service_allow=self._normalize_access_list_param(access.service.allow),
            service_deny=self._normalize_access_list_param(access.service.deny),
        )

    @staticmethod
    def _normalize_access_list_param(values: Iterable[str]) -> tuple[str, ...]:
        # rclpy/generated params treat [] string-array values as uninitialized (NOT_SET),
        # which fails parameter loading before custom validators run, so configs use [""].
        # Normalize that sentinel back to an empty list for policy wiring/extensions.
        return tuple(value.strip() for value in values if value.strip())

    def _load_runtime_config(self) -> RuntimeConfig:
        params = self._params
        return RuntimeConfig(
            subscriber=SubscriberConfig(
                max_total_subscriptions=params.subscribe.max_total_subscriptions,
                max_subscriptions_per_participant=params.subscribe.max_subscriptions_per_participant,
            ),
            publisher=PublisherConfig(
                max_topics=params.publish.max_topics,
            ),
            service=ServiceConfig(
                default_timeout_ms=params.service.timeout_ms,
                max_inflight_per_participant=params.service.max_inflight_per_participant,
            ),
            initial_backoff_ms=params.livekit.reconnect.initial_backoff_ms,
            max_backoff_ms=params.livekit.reconnect.max_backoff_ms,
        )

    def _ensure_static_access_policy_configured(
        self, access_policy: StaticAccessPolicy
    ) -> None:
        if access_policy.has_any_allow_configured():
            return
        raise ValueError(
            "No allowlists configured. The bridge is default-deny and will not permit any "
            "subscribe/publish/service operations.\n"
            "Set at least one of: access.static.subscribe.allow, access.static.publish.allow, "
            "access.static.service.allow.\n"
            'Development shortcut: set access.static.subscribe.allow: ["*"] to allow all subscriptions.'
        )

    def _load_callable_factory(self, parameter_name: str, factory_path: str) -> Any:
        factory = load_symbol(factory_path)
        if callable(factory):
            return cast(Any, factory)
        raise TypeError(
            f"{parameter_name} must resolve to a callable; got {type(factory).__name__}"
        )

    def _create_factory_instance(self, factory: Any, config: Any) -> Any:
        return cast(Any, factory)(node=self, config=config)

    def _validate_factory_result(
        self,
        *,
        parameter_name: str,
        expected_type_name: str,
        created: Any,
        required_methods: tuple[str, ...],
    ) -> None:
        for method in required_methods:
            if not callable(getattr(created, method, None)):
                raise TypeError(
                    f"{parameter_name} did not return a {expected_type_name}; "
                    f"missing .{method}() (got {type(created).__name__})"
                )

    def _load_access_factory(
        self, factory_path: str, config: AccessPolicyConfig
    ) -> AccessPolicy:
        factory = self._load_callable_factory(_ACCESS_FACTORY_PARAMETER, factory_path)
        created = self._create_factory_instance(factory, config)
        self._validate_factory_result(
            parameter_name=_ACCESS_FACTORY_PARAMETER,
            expected_type_name="AccessPolicy",
            created=created,
            required_methods=_REQUIRED_ACCESS_POLICY_METHODS,
        )
        self.get_logger().info(f"Loaded custom bridge access policy: {factory_path}")
        return cast(AccessPolicy, created)

    def _load_telemetry_factory(
        self, factory_path: str, config: RuntimeConfig
    ) -> Telemetry:
        factory = self._load_callable_factory(
            _TELEMETRY_FACTORY_PARAMETER, factory_path
        )
        created = self._create_factory_instance(factory, config)
        self._validate_factory_result(
            parameter_name=_TELEMETRY_FACTORY_PARAMETER,
            expected_type_name="Telemetry",
            created=created,
            required_methods=_REQUIRED_TELEMETRY_METHODS,
        )
        self.get_logger().info(f"Loaded custom telemetry: {factory_path}")
        return cast(Telemetry, created)


def _log_startup_failure(exc: Exception) -> None:
    try:
        from rclpy.logging import get_logger

        get_logger("livekit_bridge").error(f"LiveKit bridge failed to start: {exc}")
    except Exception:
        pass


def run_node() -> int:
    node: LivekitBridgeNode | None = None
    try:
        node = LivekitBridgeNode()
        from rclpy import spin

        spin(node)
        return 0
    except Exception as exc:
        if node is None:
            _log_startup_failure(exc)
        else:
            node.get_logger().error(f"LiveKit bridge failed: {exc}")
        return 1
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
