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

from collections.abc import Iterable

from livekit_ros2_bridge.core import access_rules
from livekit_ros2_bridge.core.access import (
    AccessOperation,
    AccessPolicy,
    AccessResource,
    AccessDecision,
)
from livekit_ros2_bridge.core.request_context import RequestContext
from livekit_ros2_bridge.core.names import normalize_ros_topic


class StaticAccessPolicy(AccessPolicy):
    """Static allow/deny policy for ROS operations.

    Rules:
    - Default-deny per operation when allowlist is empty (and allow-all is not set).
    - Denylist always wins.
    - Subscribe can enforce a ROS type denylist.
    """

    def __init__(
        self,
        *,
        publish_allow: Iterable[str] | None = None,
        publish_deny: Iterable[str] | None = None,
        subscribe_allow: Iterable[str] | None = None,
        subscribe_deny: Iterable[str] | None = None,
        subscribe_type_deny: Iterable[str] | None = None,
        service_allow: Iterable[str] | None = None,
        service_deny: Iterable[str] | None = None,
    ) -> None:
        self._publish_allow_all, self._publish_allow = access_rules.parse_allowlist(
            publish_allow
        )
        self._publish_deny = access_rules.parse_denylist(publish_deny)

        self._subscribe_allow_all, self._subscribe_allow = access_rules.parse_allowlist(
            subscribe_allow
        )
        self._subscribe_deny = access_rules.parse_denylist(subscribe_deny)
        self._subscribe_type_deny = {
            t.strip() for t in (subscribe_type_deny or []) if t.strip()
        }

        self._service_allow_all, self._service_allow = access_rules.parse_allowlist(
            service_allow
        )
        self._service_deny = access_rules.parse_denylist(service_deny)

    def has_any_allow_configured(self) -> bool:
        return bool(
            self._publish_allow_all
            or self._publish_allow
            or self._subscribe_allow_all
            or self._subscribe_allow
            or self._service_allow_all
            or self._service_allow
        )

    def authorize(
        self, ctx: RequestContext, op: AccessOperation, res: AccessResource
    ) -> AccessDecision:
        name = normalize_ros_topic(res.name)
        if not name:
            return AccessDecision(ok=False, reason="invalid_resource")

        if op == AccessOperation.PUBLISH:
            allowed = access_rules.is_allowed(
                name,
                allow_all=self._publish_allow_all,
                allowlist=self._publish_allow,
                denylist=self._publish_deny,
            )
            return AccessDecision(
                ok=allowed, reason=None if allowed else "publish_denied"
            )

        if op == AccessOperation.SUBSCRIBE:
            allowed = access_rules.is_allowed(
                name,
                allow_all=self._subscribe_allow_all,
                allowlist=self._subscribe_allow,
                denylist=self._subscribe_deny,
            )
            if not allowed:
                return AccessDecision(ok=False, reason="subscribe_denied")
            if res.ros_type and res.ros_type in self._subscribe_type_deny:
                return AccessDecision(ok=False, reason="subscribe_type_denied")
            return AccessDecision(ok=True)

        if op == AccessOperation.CALL_SERVICE:
            allowed = access_rules.is_allowed(
                name,
                allow_all=self._service_allow_all,
                allowlist=self._service_allow,
                denylist=self._service_deny,
            )
            return AccessDecision(
                ok=allowed, reason=None if allowed else "call_service_denied"
            )

        return AccessDecision(ok=False, reason="unsupported_operation")


__all__ = ["StaticAccessPolicy"]
