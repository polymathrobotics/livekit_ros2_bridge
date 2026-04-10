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
# Generate a shell script that installs rosdep-resolved dependencies.
#
# This follows the same pattern as polymathrobotics/oci's
# ros/ready/ubuntu/gather-rosdeps.sh: run rosdep in --simulate mode once, fold
# the generated install lines together, and write a reusable install script.
# This repo extends that pattern with ROSDEP_DEPENDENCY_TYPES so the Docker build
# can generate separate dev, build, and runtime install scripts from one package
# manifest.

set -euo pipefail

if (($# < 2)); then
  echo "usage: gather-rosdeps.sh <dest> <path1> [path2 ...]" >&2
  exit 1
fi

: "${ROS_DISTRO:?ROS_DISTRO must be set}"

dest="$1"
shift

mkdir -p "$(dirname "${dest}")"

rosdep_cmd=(
  rosdep install
  --from-paths "$@"
  --ignore-src
  --skip-keys "${SKIP_KEYS:-""}"
  --rosdistro "${ROS_DISTRO}"
  --default-yes
  --simulate
)

if [[ -n "${ROSDEP_DEPENDENCY_TYPES:-}" ]]; then
  # shellcheck disable=SC2206
  dependency_types=(${ROSDEP_DEPENDENCY_TYPES})
  for dependency_type in "${dependency_types[@]}"; do
    rosdep_cmd+=("--dependency-types=${dependency_type}")
  done
fi

initial="$(PIP_BREAK_SYSTEM_PACKAGES=1 "${rosdep_cmd[@]}")"

# Combine all apt-get install lines into one command so each stage performs one
# package install transaction after apt-get update.
apt_deps=$(
  printf '%s\n' "${initial}" \
    | awk '
        /^[[:space:]]*apt-get install -y / {
          sub(/^[[:space:]]*apt-get install -y /, "")
          for (i = 1; i <= NF; ++i) {
            if ($i !~ /^-/) {
              print $i
            }
          }
        }
      ' \
    | sort -u \
    | tr '\n' ' '
) || echo ''

if [[ -n "${apt_deps}" ]]; then
  apt_statement="apt-get install -y --no-install-recommends -q ${apt_deps}"
else
  apt_statement=""
fi

cat <<EOF > "${dest}"
#!/usr/bin/env bash
set -euxo pipefail
${apt_statement}
EOF

chmod +x "${dest}"
