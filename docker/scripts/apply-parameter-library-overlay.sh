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

if [ "$#" -ne 2 ]; then
  echo "usage: $0 <ros-distro> <overlay-ref>" >&2
  exit 1
fi

ros_distro="$1"
overlay_ref="$2"

python3 -m pip install --no-cache-dir --no-deps \
  "generate-parameter-library-py @ git+https://github.com/PickNikRobotics/generate_parameter_library@${overlay_ref}#subdirectory=generate_parameter_library_py"

cp /usr/local/bin/generate_parameter_library_cpp \
  "/opt/ros/${ros_distro}/bin/generate_parameter_library_cpp"

ros_py_site="$(python3 -c "import pathlib; print(next(pathlib.Path('/opt/ros/${ros_distro}').glob('lib/python*/site-packages')))")"
pip_site="$(python3 -c "import sysconfig; print(sysconfig.get_path('purelib'))")"

rm -rf "${ros_py_site}/generate_parameter_library_py"
cp -a "${pip_site}/generate_parameter_library_py" "${ros_py_site}/generate_parameter_library_py"
