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

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly REPO_ROOT
BAKE_FILE="${REPO_ROOT}/docker/bake.hcl"
readonly BAKE_FILE

usage() {
  cat <<'EOF'
usage: docker/dev-container.sh <ensure|exec|clean> [command]

ensure  Ensure the dev image, volumes, and container exist and are running.
exec    Run a command string inside the dev container workspace.
clean   Remove the dev container and its named volumes.
EOF
}

die() {
  echo "$*" >&2
  exit 1
}

require_docker() {
  command -v docker >/dev/null 2>&1 || die "docker is required"
}

ros_distro() {
  printf '%s' "${ROS_DISTRO:-humble}"
}

dev_resource_prefix() {
  printf '%s' "${DEV_RESOURCE_PREFIX:-livekit_ros2_bridge_$(ros_distro)}"
}

dev_container_name() {
  printf '%s' "$(dev_resource_prefix)"
}

dev_volume_name() {
  printf '%s_%s' "$(dev_resource_prefix)" "$1"
}

dev_target() {
  printf 'dev-%s' "$(ros_distro)"
}

dev_image() {
  printf 'livekit_ros2_bridge-builder:%s' "$(ros_distro)"
}

ensure_dev_volume() {
  local volume_name="$1"

  if ! docker volume inspect "${volume_name}" >/dev/null 2>&1; then
    docker volume create "${volume_name}" >/dev/null
  fi
}

ensure_dev_volumes() {
  ensure_dev_volume "$(dev_volume_name build)"
  ensure_dev_volume "$(dev_volume_name install)"
  ensure_dev_volume "$(dev_volume_name logs)"
  ensure_dev_volume "$(dev_volume_name ccache)"
}

validate_dev_target() {
  docker buildx bake --file "${BAKE_FILE}" --print "$(dev_target)" >/dev/null
}

ensure_dev_image() {
  validate_dev_target

  if ! docker image inspect "$(dev_image)" >/dev/null 2>&1; then
    docker buildx bake --file "${BAKE_FILE}" --load "$(dev_target)"
  fi
}

remove_dev_container() {
  local container_name

  container_name="$(dev_container_name)"
  if docker container inspect "${container_name}" >/dev/null 2>&1; then
    docker rm -f "${container_name}" >/dev/null
  fi
}

start_dev_container() {
  docker run -d \
    --platform linux/amd64 \
    --init \
    --name "$(dev_container_name)" \
    --workdir /workspace \
    --env ROS_DISTRO="$(ros_distro)" \
    --env CCACHE_DIR=/ccache \
    --volume "${REPO_ROOT}:/workspace" \
    --volume "$(dev_volume_name build):/workspace/build" \
    --volume "$(dev_volume_name install):/workspace/install" \
    --volume "$(dev_volume_name logs):/workspace/log" \
    --volume "$(dev_volume_name ccache):/ccache" \
    "$(dev_image)" \
    sleep infinity >/dev/null
}

ensure_dev_container() {
  local container_name

  ensure_dev_image
  ensure_dev_volumes

  container_name="$(dev_container_name)"
  if ! docker container inspect "${container_name}" >/dev/null 2>&1; then
    start_dev_container
    return
  fi

  if [[ "$(docker inspect -f '{{.State.Running}}' "${container_name}")" != "true" ]]; then
    docker start "${container_name}" >/dev/null
  fi
}

exec_in_dev_container() {
  local command="$1"

  ensure_dev_container
  docker exec -i \
    --env ROS_DISTRO="$(ros_distro)" \
    "$(dev_container_name)" \
    bash -lc '
      set -euo pipefail
      cd /workspace
      set +u
      source "/opt/ros/${ROS_DISTRO}/setup.bash"
      if [[ -f /opt/polymath/setup.bash ]]; then
        source /opt/polymath/setup.bash
      fi
      if [[ -f /workspace/install/setup.bash ]]; then
        source /workspace/install/setup.bash
      fi
      set -u
      export GST_PLUGIN_PATH="/workspace/install/lib/livekit_ros2_bridge${GST_PLUGIN_PATH:+:${GST_PLUGIN_PATH}}"
      export CCACHE_DIR="${CCACHE_DIR:-/ccache}"
      eval "$1"
    ' bash "${command}"
}

clean_dev_state() {
  remove_dev_container
  docker volume rm -f \
    "$(dev_volume_name build)" \
    "$(dev_volume_name install)" \
    "$(dev_volume_name logs)" \
    "$(dev_volume_name ccache)" >/dev/null 2>&1 || true
}

main() {
  local subcommand="${1:-}"

  require_docker
  case "${subcommand}" in
    ensure)
      shift
      (($# == 0)) || die "ensure does not take arguments"
      ensure_dev_container
      ;;
    exec)
      shift
      (($# == 1)) || die "exec requires exactly one command string"
      exec_in_dev_container "$1"
      ;;
    clean)
      shift
      (($# == 0)) || die "clean does not take arguments"
      clean_dev_state
      ;;
    ""|help|-h|--help)
      usage
      ;;
    *)
      usage >&2
      die "unknown subcommand: ${subcommand}"
      ;;
  esac
}

main "$@"
