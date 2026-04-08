set dotenv-load := true
set export := true
set positional-arguments := true

shell_common := '''
export ROS_DISTRO="${ROS_DISTRO:-humble}"

dev_resource_prefix() {
  printf '%s' "${DEV_RESOURCE_PREFIX:-${COMPOSE_PROJECT_NAME:-livekit_ros2_bridge_${ROS_DISTRO}}}"
}

dev_container_name() {
  printf '%s' "$(dev_resource_prefix)"
}

dev_volume_name() {
  printf '%s_%s' "$(dev_resource_prefix)" "$1"
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

bake() {
  docker buildx bake \
    --file "${bake_file}" \
    "$@"
}

dev_target() {
  printf 'dev-%s' "${ROS_DISTRO}"
}

dev_image() {
  printf 'livekit_ros2_bridge-builder:%s' "${ROS_DISTRO}"
}

validate_bake_ref() {
  bake --print "$1" >/dev/null
}

build_bake_ref() {
  local ref="$1"
  local load="${2:-1}"
  local no_cache="${3:-0}"
  local args=()

  validate_bake_ref "${ref}"

  if [[ "${load}" == "1" ]]; then
    args+=(--load)
  fi
  if [[ "${no_cache}" == "1" ]]; then
    args+=(--no-cache)
  fi

  bake "${args[@]}" "${ref}"
}

ensure_dev_image() {
  local image_ref
  image_ref="$(dev_image)"

  if ! docker image inspect "${image_ref}" >/dev/null 2>&1; then
    build_bake_ref "$(dev_target)" 1 0
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
  local container_name
  container_name="$(dev_container_name)"

  docker run -d \
    --platform linux/amd64 \
    --init \
    --name "${container_name}" \
    --workdir /workspace \
    --env ROS_DISTRO="${ROS_DISTRO}" \
    --env CCACHE_DIR=/ccache \
    --volume "${repo_root}:/workspace" \
    --volume "$(dev_volume_name build):/workspace/build" \
    --volume "$(dev_volume_name install):/workspace/install" \
    --volume "$(dev_volume_name logs):/workspace/log" \
    --volume "$(dev_volume_name ccache):/ccache" \
    "$(dev_image)" \
    sleep infinity >/dev/null
}

ensure_dev_container() {
  local force_recreate=0
  local arg

  for arg in "$@"; do
    case "${arg}" in
      --force-recreate)
        force_recreate=1
        ;;
      *)
        echo "unsupported ensure_dev_container arg: ${arg}" >&2
        exit 1
        ;;
    esac
  done

  ensure_dev_image
  ensure_dev_volumes

  if (( force_recreate )); then
    remove_dev_container
  fi

  local container_name
  container_name="$(dev_container_name)"

  if ! docker container inspect "${container_name}" >/dev/null 2>&1; then
    start_dev_container
    return
  fi

  if [[ "$(docker inspect -f '{{.State.Running}}' "${container_name}")" != "true" ]]; then
    docker start "${container_name}" >/dev/null
  fi
}

dev_bash() {
  ensure_dev_container
  docker exec -i "$(dev_container_name)" bash -lc "$1"
}

remove_dev_volumes() {
  docker volume rm -f \
    "$(dev_volume_name build)" \
    "$(dev_volume_name install)" \
    "$(dev_volume_name logs)" \
    "$(dev_volume_name ccache)" >/dev/null 2>&1 || true
}
'''

default:
    @just --list

image-build *args:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="{{ justfile_directory() }}"
    bake_file="${repo_root}/docker-bake.hcl"
    {{ shell_common }}
    if (($# != 2)); then
      echo "usage: just image-build <dev|runtime> <distro>" >&2
      exit 1
    fi
    build_bake_ref "$1-$2" 1 0

image-build-all *args:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="{{ justfile_directory() }}"
    bake_file="${repo_root}/docker-bake.hcl"
    {{ shell_common }}
    if (($# > 1)); then
      echo "usage: just image-build-all [group]" >&2
      exit 1
    fi
    build_bake_ref "${1:-all}" 1 0

image-print *args:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="{{ justfile_directory() }}"
    bake_file="${repo_root}/docker-bake.hcl"
    {{ shell_common }}
    if (($# == 0)); then
      set -- all
    fi
    bake --print "$@"

up:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="{{ justfile_directory() }}"
    bake_file="${repo_root}/docker-bake.hcl"
    {{ shell_common }}
    ensure_dev_container
    echo "Dev container ready for ROS_DISTRO=${ROS_DISTRO}."

up-build:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="{{ justfile_directory() }}"
    bake_file="${repo_root}/docker-bake.hcl"
    {{ shell_common }}
    build_bake_ref "$(dev_target)" 1 0
    ensure_dev_container --force-recreate
    echo "Dev container ready for ROS_DISTRO=${ROS_DISTRO}."

up-no-cache:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="{{ justfile_directory() }}"
    bake_file="${repo_root}/docker-bake.hcl"
    {{ shell_common }}
    build_bake_ref "$(dev_target)" 1 1
    ensure_dev_container --force-recreate
    echo "Dev container ready for ROS_DISTRO=${ROS_DISTRO}."

build *colcon_args:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="{{ justfile_directory() }}"
    bake_file="${repo_root}/docker-bake.hcl"
    {{ shell_common }}
    extra_args=()
    force_configure=0
    for arg in "$@"; do
      case "${arg}" in
        --force-configure)
          force_configure=1
          ;;
        *)
          extra_args+=("$(printf '%q' "${arg}")")
          ;;
      esac
    done

    cmake_force_configure_arg=""
    if (( force_configure )); then
      cmake_force_configure_arg="--cmake-force-configure"
    fi

    extra_args_str=""
    if ((${#extra_args[@]})); then
      extra_args_str="${extra_args[*]}"
    fi

    dev_bash "
      source /usr/local/bin/livekit-bridge-dev-env
      cd /workspace
      mkdir -p /workspace/build /workspace/install
      colcon \
        --log-base /workspace/log \
        build \
        --merge-install \
        --packages-up-to livekit_ros2_bridge \
        --build-base /workspace/build \
        --install-base /workspace/install \
        ${cmake_force_configure_arg} \
        --cmake-args -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        ${extra_args_str}
    "

build-force-configure *colcon_args:
    #!/usr/bin/env bash
    set -euo pipefail
    exec just build --force-configure "$@"

test *colcon_args:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="{{ justfile_directory() }}"
    bake_file="${repo_root}/docker-bake.hcl"
    {{ shell_common }}
    extra_args=()
    for arg in "$@"; do
      extra_args+=("$(printf '%q' "${arg}")")
    done

    extra_args_str=""
    if ((${#extra_args[@]})); then
      extra_args_str="${extra_args[*]}"
    fi

    dev_bash "
      source /usr/local/bin/livekit-bridge-dev-env
      cd /workspace
      rm -rf /workspace/build/livekit_ros2_bridge/test_results
      colcon \
        --log-base /workspace/log \
        test \
        --merge-install \
        --packages-select livekit_ros2_bridge \
        --build-base /workspace/build \
        --install-base /workspace/install \
        --return-code-on-test-failure \
        ${extra_args_str}
      colcon \
        --log-base /workspace/log \
        test-result \
        --verbose
    "

reset:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="{{ justfile_directory() }}"
    bake_file="${repo_root}/docker-bake.hcl"
    {{ shell_common }}
    remove_dev_container
    remove_dev_volumes
    echo "Reset complete for ROS_DISTRO=${ROS_DISTRO}."

reset-rebuild-image:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="{{ justfile_directory() }}"
    bake_file="${repo_root}/docker-bake.hcl"
    {{ shell_common }}
    remove_dev_container
    remove_dev_volumes
    build_bake_ref "$(dev_target)" 1 1
    echo "Reset complete for ROS_DISTRO=${ROS_DISTRO}."
