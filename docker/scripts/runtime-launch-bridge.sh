#!/bin/bash
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
set -euo pipefail

: "${ROS_DISTRO:?ROS_DISTRO must be set}"
: "${LIVEKIT_URL:?LIVEKIT_URL must be set}"

room_name="${LIVEKIT_BRIDGE_ROOM_NAME:-${POLYMATH_LIVEKIT_ROOM_NAME:-}}"
bridge_identity="${LIVEKIT_BRIDGE_IDENTITY:-${POLYMATH_LIVEKIT_BRIDGE_IDENTITY:-}}"
livekit_token="${LIVEKIT_BRIDGE_TOKEN:-${LIVEKIT_TOKEN:-}}"
api_key="${LIVEKIT_API_KEY:-}"
api_secret="${LIVEKIT_API_SECRET:-}"
subscribe_allowlist="${LIVEKIT_BRIDGE_SUBSCRIBE_ALLOWLIST:-}"
publish_allowlist="${LIVEKIT_BRIDGE_PUBLISH_ALLOWLIST:-}"
service_allowlist="${LIVEKIT_BRIDGE_SERVICE_ALLOWLIST:-${POLYMATH_LIVEKIT_SERVICE_ALLOWLIST:-}}"

: "${room_name:?POLYMATH_LIVEKIT_ROOM_NAME or LIVEKIT_BRIDGE_ROOM_NAME must be set}"
: "${bridge_identity:?POLYMATH_LIVEKIT_BRIDGE_IDENTITY or LIVEKIT_BRIDGE_IDENTITY must be set}"

if [[ -z "${livekit_token}" && ( -z "${api_key}" || -z "${api_secret}" ) ]]; then
  echo "[livekit-bridge-launch] ERROR: set LIVEKIT_BRIDGE_TOKEN/LIVEKIT_TOKEN or LIVEKIT_API_KEY and LIVEKIT_API_SECRET" >&2
  exit 1
fi

if [[ -z "${subscribe_allowlist}${publish_allowlist}${service_allowlist}" ]]; then
  echo "[livekit-bridge-launch] ERROR: configure at least one bridge allowlist env var" >&2
  exit 1
fi

trim() {
  local value="$1"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "${value}"
}

json_quote() {
  local value="$1"
  value="${value//\\/\\\\}"
  value="${value//\"/\\\"}"
  printf '"%s"' "${value}"
}

render_string_array() {
  local raw="$1"
  local output="["
  local first=1
  local item=""

  IFS=',' read -r -a items <<< "${raw}"
  for item in "${items[@]}"; do
    item="$(trim "${item}")"
    if [[ -z "${item}" ]]; then
      continue
    fi

    if (( first == 0 )); then
      output+=", "
    fi
    output+="$(json_quote "${item}")"
    first=0
  done

  output+="]"
  printf '%s' "${output}"
}

source /usr/local/bin/livekit-bridge-source-env /opt/livekit_ros2_bridge/install

params_file=/tmp/livekit_bridge.params.yaml

cat > "${params_file}" <<EOF
livekit_ros2_bridge:
  ros__parameters:
    livekit.url: $(json_quote "${LIVEKIT_URL}")
    livekit.room: $(json_quote "${room_name}")
    livekit.identity: $(json_quote "${bridge_identity}")
    livekit.token: $(json_quote "${livekit_token}")
    livekit.api_key: $(json_quote "${api_key}")
    livekit.api_secret: $(json_quote "${api_secret}")
    access.rules.subscribe.allow: $(render_string_array "${subscribe_allowlist}")
    access.rules.publish.allow: $(render_string_array "${publish_allowlist}")
    access.rules.service.allow: $(render_string_array "${service_allowlist}")
EOF

exec /opt/livekit_ros2_bridge/install/lib/livekit_ros2_bridge/livekit_ros2_bridge_node \
  --ros-args \
  --params-file "${params_file}"
