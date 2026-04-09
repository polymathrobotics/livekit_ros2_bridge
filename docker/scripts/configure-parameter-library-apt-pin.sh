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

if [ "$#" -ne 1 ]; then
  echo "usage: $0 <ros-distro>" >&2
  exit 1
fi

ros_distro="$1"

case "${ros_distro}" in
  jazzy|rolling)
    ;;
  *)
    exit 0
    ;;
esac

. /etc/os-release

cat > /etc/apt/sources.list.d/ros2-testing.list <<EOF
deb [signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg] http://packages.ros.org/ros2-testing/ubuntu ${UBUNTU_CODENAME} main
EOF

cat > /etc/apt/preferences.d/generate-parameter-library.pref <<EOF
# Standard ROS main currently lags the released generate_parameter_library
# packages for ${ros_distro}. Prefer the stable main repository by default,
# then selectively raise only the generate_parameter_library package family from
# ros2-testing until the main repository catches up.
Package: *
Pin: release o=ROS,n=${UBUNTU_CODENAME},l=ROS 2,a=testing
Pin-Priority: 100

Package: *
Pin: release o=ROS,n=${UBUNTU_CODENAME},l=ROS 2,a=stable
Pin-Priority: 500

Package: ros-${ros_distro}-generate-parameter-library
Pin: release o=ROS,n=${UBUNTU_CODENAME},l=ROS 2,a=testing
Pin-Priority: 990

Package: ros-${ros_distro}-generate-parameter-library-py
Pin: release o=ROS,n=${UBUNTU_CODENAME},l=ROS 2,a=testing
Pin-Priority: 990

Package: ros-${ros_distro}-parameter-traits
Pin: release o=ROS,n=${UBUNTU_CODENAME},l=ROS 2,a=testing
Pin-Priority: 990
EOF
