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
# Configure ROS apt sources needed by the Docker images.
#
# Newer ROS distros still need generate_parameter_library packages from
# ros2-testing, but we do not want the rest of ROS to drift off the stable
# repository. This helper adds the testing source and pins only that package
# family so every stage can share the same apt setup.

set -euo pipefail

: "${ROS_DISTRO:?ROS_DISTRO must be set}"

case "${ROS_DISTRO}" in
  jazzy|kilted|rolling)
    . /etc/os-release

    cat > /etc/apt/sources.list.d/ros2-testing.list <<APT_LIST
deb [signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg] http://packages.ros.org/ros2-testing/ubuntu ${UBUNTU_CODENAME} main
APT_LIST

    cat > /etc/apt/preferences.d/generate-parameter-library.pref <<APT_PREF
# Standard ROS main currently lags the released generate_parameter_library
# packages for ${ROS_DISTRO}. Prefer the stable main repository by default,
# then selectively raise only the generate_parameter_library package family from
# ros2-testing until the main repository catches up.
Package: *
Pin: release o=ROS,n=${UBUNTU_CODENAME},l=ROS 2,a=testing
Pin-Priority: 100

Package: *
Pin: release o=ROS,n=${UBUNTU_CODENAME},l=ROS 2,a=stable
Pin-Priority: 500

Package: ros-${ROS_DISTRO}-generate-parameter-library
Pin: release o=ROS,n=${UBUNTU_CODENAME},l=ROS 2,a=testing
Pin-Priority: 990

Package: ros-${ROS_DISTRO}-generate-parameter-library-py
Pin: release o=ROS,n=${UBUNTU_CODENAME},l=ROS 2,a=testing
Pin-Priority: 990

Package: ros-${ROS_DISTRO}-parameter-traits
Pin: release o=ROS,n=${UBUNTU_CODENAME},l=ROS 2,a=testing
Pin-Priority: 990
APT_PREF
    ;;
esac
