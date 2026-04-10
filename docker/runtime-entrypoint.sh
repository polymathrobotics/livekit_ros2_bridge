#!/usr/bin/env bash

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
# Default entrypoint for the packaged runtime image.
#
# With no arguments, it sources the ROS and package environments and launches
# the bridge against the mounted params file. If a command is provided, it runs
# that command inside the same prepared runtime environment instead.

set -euo pipefail

: "${ROS_DISTRO:?ROS_DISTRO must be set}"

restore_nounset() {
  if [[ "${1}" == "1" ]]; then
    set -u
  fi
}

source_bridge_env() {
  local install_prefix="${1:-}"
  local nounset_was_enabled=0

  case $- in
    *u*) nounset_was_enabled=1 ;;
  esac

  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ -f /opt/polymath/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/polymath/setup.bash
  fi
  if [[ -n "${install_prefix}" && -f "${install_prefix}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "${install_prefix}/setup.bash"
  fi
  restore_nounset "${nounset_was_enabled}"

}

if (($#)); then
  source_bridge_env /opt/livekit_ros2_bridge/install
  exec "$@"
fi

params_file="${LIVEKIT_BRIDGE_PARAMS_FILE:-/config/livekit_bridge.params.yaml}"
if [[ ! -r "${params_file}" ]]; then
  echo "[livekit-bridge-launch] ERROR: params file '${params_file}' is not readable; mount a local params file or override the command" >&2
  exit 1
fi

source_bridge_env /opt/livekit_ros2_bridge/install

exec /opt/livekit_ros2_bridge/install/lib/livekit_ros2_bridge/livekit_ros2_bridge_node \
  --ros-args \
  --params-file "${params_file}"
