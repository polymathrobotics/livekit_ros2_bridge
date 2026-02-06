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
from typing import Any, cast

from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter

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

_STRING_ARRAY_PARAMETER_NAMES: tuple[str, ...] = (
    "access.static.subscribe.allow",
    "access.static.subscribe.deny",
    "access.static.publish.allow",
    "access.static.publish.deny",
    "access.static.service.allow",
    "access.static.service.deny",
)

_DEFAULT_SUBSCRIBE_TYPE_DENYLIST: tuple[str, ...] = (
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/PointCloud2",
)

_REQUIRED_ACCESS_POLICY_METHODS: tuple[str, ...] = ("authorize",)
_REQUIRED_TELEMETRY_METHODS: tuple[str, ...] = ("emit",)


class LivekitBridgeNode(Node):
    """ROS2 node that bridges ROS topics/services over LiveKit."""

    def __init__(self) -> None:
        super().__init__("livekit_bridge")
        self._runtime: Runtime | None = None

        self._declare_bridge_parameters()

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

    def _declare_bridge_parameters(self) -> None:
        self._declare_scalar_parameters(
            (
                ("livekit.url", ""),
                ("livekit.room", ""),
                ("livekit.identity", ""),
                ("livekit.token", ""),
                ("livekit.api_key", ""),
                ("livekit.api_secret", ""),
                ("livekit.token_ttl_seconds", 3600),
                ("livekit.reconnect.initial_backoff_ms", 500),
                ("livekit.reconnect.max_backoff_ms", 10000),
            )
        )
        self._declare_scalar_parameters(
            (
                ("access.factory", ""),
                ("telemetry.factory", ""),
            )
        )

        # NOTE: rclpy infers `[]` as BYTE_ARRAY. To support overrides like
        # `access.static.subscribe.allow: ["*"]`, declare empty string arrays by type.
        self.declare_parameters(
            "",
            [
                (name, Parameter.Type.STRING_ARRAY)
                for name in _STRING_ARRAY_PARAMETER_NAMES
            ],
        )
        self.declare_parameter(
            "access.static.subscribe.type_deny", list(_DEFAULT_SUBSCRIBE_TYPE_DENYLIST)
        )

        self._declare_scalar_parameters(
            (
                ("subscribe.max_total_subscriptions", 128),
                ("subscribe.max_subscriptions_per_participant", 32),
                ("publish.max_topics", 50),
                ("service.timeout_ms", 2000),
                ("service.max_inflight_per_participant", 4),
            )
        )

    def _declare_scalar_parameters(
        self, definitions: tuple[tuple[str, str | int | bool], ...]
    ) -> None:
        for name, default in definitions:
            self.declare_parameter(name, default)

    def _get_param_value(self, name: str) -> Any | None:
        try:
            return self.get_parameter(name).value
        except ParameterUninitializedException:
            return None

    def _get_param_str(self, name: str) -> str | None:
        value = self._get_param_value(name)
        if value is None:
            return None
        if isinstance(value, str):
            value = value.strip()
            return value or None
        self.get_logger().warning(
            f"Parameter '{name}' should be a string; got {type(value).__name__}"
        )
        return None

    def _get_param_int(self, name: str, default: int) -> int:
        value = self._get_param_value(name)
        if value is None:
            return default
        if isinstance(value, int):
            return value if value >= 0 else default
        if isinstance(value, str):
            try:
                parsed = int(value.strip())
                return parsed if parsed >= 0 else default
            except ValueError:
                self.get_logger().warning(
                    f"Parameter '{name}' should be an int; got '{value}'"
                )
                return default
        self.get_logger().warning(
            f"Parameter '{name}' should be an int; got {type(value).__name__}"
        )
        return default

    def _get_param_str_list(self, name: str) -> list[str]:
        value = self._get_param_value(name)
        if value is None:
            return []
        if isinstance(value, list):
            items: list[str] = []
            for entry in value:
                token = str(entry).strip()
                if token:
                    items.append(token)
            return items
        if isinstance(value, str):
            return [token.strip() for token in value.split(",") if token.strip()]
        self.get_logger().warning(
            f"Parameter '{name}' should be a list of strings; got {type(value).__name__}"
        )
        return []

    def _build_access_policy(self, config: AccessPolicyConfig) -> AccessPolicy:
        factory_path = self._get_param_str("access.factory")
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
        factory_path = self._get_param_str("telemetry.factory")
        if not factory_path:
            return NullTelemetry()
        return self._load_telemetry_factory(factory_path, config)

    def _load_livekit_config(self) -> tuple[LivekitConnectConfig, TokenSource]:
        url = self._get_param_str("livekit.url")
        room = self._get_param_str("livekit.room")
        identity = self._get_param_str("livekit.identity")
        token = self._get_param_str("livekit.token")
        api_key = self._get_param_str("livekit.api_key")
        api_secret = self._get_param_str("livekit.api_secret")
        ttl_seconds = self._get_param_int("livekit.token_ttl_seconds", 3600)

        if not url:
            raise ValueError("livekit.url is required")
        if not room:
            raise ValueError("livekit.room is required")

        if not identity:
            identity = f"{self.get_name()}-{socket.gethostname()}"

        if token:
            token_source: TokenSource = StaticTokenSource(token=token)
            token_source_name = "static token"
        elif api_key and api_secret:
            token_source = ApiKeyTokenSource(
                api_key=api_key,
                api_secret=api_secret,
                ttl_seconds=ttl_seconds,
            )
            token_source_name = "api key/secret"
        else:
            raise ValueError(
                "Either livekit.token or livekit.api_key + livekit.api_secret must be set"
            )

        self.get_logger().info(
            f"LiveKit config loaded: url={url} room={room} identity={identity} "
            f"token_source={token_source_name} ttl_seconds={ttl_seconds}"
        )

        return LivekitConnectConfig(url=url, room=room, identity=identity), token_source

    def _load_access_config(self) -> AccessPolicyConfig:
        return AccessPolicyConfig(
            publish_allow=self._get_param_str_list("access.static.publish.allow"),
            publish_deny=self._get_param_str_list("access.static.publish.deny"),
            subscribe_allow=self._get_param_str_list("access.static.subscribe.allow"),
            subscribe_deny=self._get_param_str_list("access.static.subscribe.deny"),
            subscribe_type_deny=self._get_param_str_list(
                "access.static.subscribe.type_deny"
            ),
            service_allow=self._get_param_str_list("access.static.service.allow"),
            service_deny=self._get_param_str_list("access.static.service.deny"),
        )

    def _load_runtime_config(self) -> RuntimeConfig:
        return RuntimeConfig(
            subscriber=SubscriberConfig(
                max_total_subscriptions=self._get_param_int(
                    "subscribe.max_total_subscriptions",
                    128,
                ),
                max_subscriptions_per_participant=self._get_param_int(
                    "subscribe.max_subscriptions_per_participant",
                    32,
                ),
            ),
            publisher=PublisherConfig(
                max_topics=self._get_param_int("publish.max_topics", 50),
            ),
            service=ServiceConfig(
                default_timeout_ms=self._get_param_int("service.timeout_ms", 2000),
                max_inflight_per_participant=self._get_param_int(
                    "service.max_inflight_per_participant",
                    4,
                ),
            ),
            initial_backoff_ms=self._get_param_int(
                "livekit.reconnect.initial_backoff_ms", 500
            ),
            max_backoff_ms=self._get_param_int(
                "livekit.reconnect.max_backoff_ms", 10000
            ),
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
        factory = self._load_callable_factory("access.factory", factory_path)
        created = self._create_factory_instance(factory, config)
        self._validate_factory_result(
            parameter_name="access.factory",
            expected_type_name="AccessPolicy",
            created=created,
            required_methods=_REQUIRED_ACCESS_POLICY_METHODS,
        )
        self.get_logger().info(f"Loaded custom bridge access policy: {factory_path}")
        return cast(AccessPolicy, created)

    def _load_telemetry_factory(
        self, factory_path: str, config: RuntimeConfig
    ) -> Telemetry:
        factory = self._load_callable_factory("telemetry.factory", factory_path)
        created = self._create_factory_instance(factory, config)
        self._validate_factory_result(
            parameter_name="telemetry.factory",
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
            return 1
        node.get_logger().error(f"LiveKit bridge failed: {exc}")
        return 1
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
