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

source /usr/local/bin/livekit-bridge-source-env /opt/livekit_ros2_bridge/install

gst-inspect-1.0 rosrawimagesrc >/dev/null
gst-inspect-1.0 roscompressedimagesrc >/dev/null
gstreamer-publisher --help >/dev/null
test -s /etc/ssl/certs/ca-certificates.crt

status=0
/usr/bin/timeout 5s /opt/livekit_ros2_bridge/install/lib/livekit_ros2_bridge/livekit_ros2_bridge_node \
  --ros-args \
  -p livekit.url:=ws://test:7880 \
  -p livekit.room:=test-room \
  -p livekit.token:=test-token \
  >/tmp/livekit-bridge-runtime-smoke.log 2>&1 || status=$?

if [ "${status}" -ne 124 ]; then
  cat /tmp/livekit-bridge-runtime-smoke.log
  exit "${status}"
fi
