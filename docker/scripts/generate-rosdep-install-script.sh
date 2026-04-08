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

if [ "$#" -lt 2 ]; then
  echo "usage: $0 <output> <from-path> [<from-path> ...]" >&2
  exit 1
fi

output=$1
shift

rosdep_cmd=(
  rosdep install
  --from-paths "$@"
  --ignore-src
  --skip-keys "${SKIP_KEYS:-}"
  --rosdistro "${ROS_DISTRO:?ROS_DISTRO must be set}"
  --default-yes
  --simulate
)

if [ -n "${DEPENDENCY_TYPES:-}" ]; then
  rosdep_cmd+=(--dependency-types "${DEPENDENCY_TYPES}")
fi

simulation_output="$("${rosdep_cmd[@]}")"

apt_deps="$(
  printf '%s\n' "${simulation_output}" \
    | grep "apt-get install" \
    | sed 's/'\''\(apt-get install -y\)\(.*\)'\'' .*/\1\2/g' \
    | awk '{print $NF}' \
    | sort -u \
    | tr '\n' ' ' \
    || true
)"

pip_deps="$(
  printf '%s\n' "${simulation_output}" \
    | grep "pip3 install" \
    | awk '{print $NF}' \
    | sort -u \
    | tr '\n' ' ' \
    || true
)"

cat <<EOF > "${output}"
#!/bin/bash
set -euxo pipefail
${apt_deps:+apt-get install -y --no-install-recommends -q ${apt_deps}}
${pip_deps:+PIP_BREAK_SYSTEM_PACKAGES=1 python3 -m pip install ${pip_deps}}
EOF

chmod +x "${output}"
