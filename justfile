set dotenv-load := true
set export := true
set positional-arguments := true

default:
    @just --list

build *args:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="{{ justfile_directory() }}"
    sdk_distro=noble
    if [[ "${ROS_DISTRO:-humble}" == "humble" ]]; then
      sdk_distro=jammy
    fi
    command="mkdir -p /workspace/build /workspace/install && colcon --log-base /workspace/log build --merge-install --packages-up-to livekit_ros2_bridge --build-base /workspace/build --install-base /workspace/install --cmake-args -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DLIVEKIT_SDK_DISTRO=${sdk_distro}"
    if (($#)); then
      for arg in "$@"; do
        command+=" $(printf '%q' "${arg}")"
      done
    fi
    "${repo_root}/docker/dev-container.sh" exec "${command}"

test *args:
    #!/usr/bin/env bash
    set -euo pipefail
    cd "{{ justfile_directory() }}"
    just build
    repo_root="{{ justfile_directory() }}"
    command='rm -rf /workspace/build/livekit_ros2_bridge/test_results && colcon --log-base /workspace/log test --merge-install --packages-select livekit_ros2_bridge --build-base /workspace/build --install-base /workspace/install --return-code-on-test-failure'
    if (($#)); then
      for arg in "$@"; do
        command+=" $(printf '%q' "${arg}")"
      done
    fi
    command+=' && colcon --log-base /workspace/log test-result --verbose'
    "${repo_root}/docker/dev-container.sh" exec "${command}"

test-matrix *distros:
    #!/usr/bin/env bash
    set -euo pipefail
    repo_root="{{ justfile_directory() }}"
    bake_file="${repo_root}/docker/bake.hcl"
    targets=()
    if (($# == 0)); then
      targets+=(test-all)
    else
      for distro in "$@"; do
        targets+=("test-${distro}")
      done
    fi
    docker buildx bake --file "${bake_file}" --progress plain "${targets[@]}"

format:
    #!/usr/bin/env bash
    set -euo pipefail
    cd "{{ justfile_directory() }}"
    pre-commit run --all-files

clean:
    #!/usr/bin/env bash
    set -euo pipefail
    "{{ justfile_directory() }}/docker/dev-container.sh" clean
