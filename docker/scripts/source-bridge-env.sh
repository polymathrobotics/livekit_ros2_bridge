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
: "${ROS_DISTRO:?ROS_DISTRO must be set}"

restore_nounset() {
  if [[ "${1}" == "1" ]]; then
    set -u
  fi
}

install_prefix="${1:-}"
nounset_was_enabled=0

case $- in
  *u*) nounset_was_enabled=1 ;;
esac

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f /opt/polymath/setup.bash ]]; then
  source /opt/polymath/setup.bash
fi
if [[ -n "${install_prefix}" && -f "${install_prefix}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${install_prefix}/setup.bash"
fi
restore_nounset "${nounset_was_enabled}"

if [[ -n "${install_prefix}" ]]; then
  export GST_PLUGIN_PATH="${install_prefix}/lib/livekit_ros2_bridge${GST_PLUGIN_PATH:+:${GST_PLUGIN_PATH}}"
fi
