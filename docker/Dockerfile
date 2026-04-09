# syntax=docker/dockerfile:1

ARG ROS_DISTRO=humble
ARG ROS_BUILDER_IMAGE=polymathrobotics/ros:${ROS_DISTRO}-builder-ubuntu
ARG ROS_RUNTIME_IMAGE=polymathrobotics/ros:${ROS_DISTRO}-ready-ubuntu
ARG GSTREAMER_PUBLISHER_IMAGE=docker.io/polymathrobotics/gstreamer-publisher:jammy-8825fee6f40ff51f2cf9347892f6fbc08eeb1f2e
ARG LIVEKIT_SDK_DISTRO=
ARG LIVEKIT_SDK_URL_OVERRIDE=
ARG LIVEKIT_SDK_SHA256_OVERRIDE=

FROM ${GSTREAMER_PUBLISHER_IMAGE} AS gstreamer-publisher-source

FROM ${ROS_BUILDER_IMAGE} AS ros-builder-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

RUN bash <<'EOF'
set -euxo pipefail

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

rosdep update --rosdistro "${ROS_DISTRO}"
EOF

FROM ros-builder-base AS dev

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO

ENV CCACHE_DIR=/ccache

WORKDIR /opt/dev-bootstrap

COPY package.xml /opt/dev-bootstrap/src/livekit_ros2_bridge/package.xml

# Jazzy, Kilted, and Rolling still lag on packages.ros.org main for the
# generate_parameter_library family. Add ros2-testing, but pin only those
# packages so the rest of ROS continues to come from the stable main repo.
RUN bash <<'EOF'
set -euxo pipefail

rosdep_cmd=(
  rosdep install
  --from-paths /opt/dev-bootstrap/src
  --ignore-src
  --skip-keys "${SKIP_KEYS:-}"
  --rosdistro "${ROS_DISTRO:?ROS_DISTRO must be set}"
  --default-yes
  --simulate
)
simulation_output="$("${rosdep_cmd[@]}")"

printf '%s\n' "${simulation_output}" \
  | grep "apt-get install" \
  | sed "s/'\\(apt-get install -y\\)\\(.*\\)' .*/\\1\\2/g" \
  | awk '{print $NF}' \
  | sort -u > /tmp/dev-apt-deps.txt || true

printf '%s\n' "${simulation_output}" \
  | grep "pip3 install" \
  | awk '{print $NF}' \
  | sort -u > /tmp/dev-pip-deps.txt || true

apt-get update
xargs -r apt-get install -y --no-install-recommends -q < /tmp/dev-apt-deps.txt
xargs -r env PIP_BREAK_SYSTEM_PACKAGES=1 python3 -m pip install < /tmp/dev-pip-deps.txt
apt-get install -y --no-install-recommends \
  ca-certificates \
  ccache \
  git \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer1.0-dev \
  pkg-config \
  ros-${ROS_DISTRO}-ros2cli \
  ros-${ROS_DISTRO}-ros2cli-common-extensions
rm -rf /var/lib/apt/lists/*
EOF

COPY --from=gstreamer-publisher-source /usr/local/bin/gstreamer-publisher /usr/local/bin/gstreamer-publisher

WORKDIR /workspace

CMD ["sleep", "infinity"]

FROM ros-builder-base AS builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO
ARG LIVEKIT_SDK_DISTRO
ARG LIVEKIT_SDK_URL_OVERRIDE
ARG LIVEKIT_SDK_SHA256_OVERRIDE

WORKDIR /opt/ros_ws

COPY package.xml src/livekit_ros2_bridge/package.xml

# Jazzy, Kilted, and Rolling still lag on packages.ros.org main for the
# generate_parameter_library family. Add ros2-testing, but pin only those
# packages so the rest of ROS continues to come from the stable main repo.
RUN bash <<'EOF'
set -euxo pipefail

write_rosdep_manifests() {
  local prefix="$1"
  local dependency_types="${2:-}"
  local rosdep_cmd=(
    rosdep install
    --from-paths src
    --ignore-src
    --skip-keys "${SKIP_KEYS:-}"
    --rosdistro "${ROS_DISTRO:?ROS_DISTRO must be set}"
    --default-yes
    --simulate
  )
  local simulation_output

  if [[ -n "${dependency_types}" ]]; then
    rosdep_cmd+=(--dependency-types "${dependency_types}")
  fi

  simulation_output="$("${rosdep_cmd[@]}")"

  printf '%s\n' "${simulation_output}" \
    | grep "apt-get install" \
    | sed "s/'\\(apt-get install -y\\)\\(.*\\)' .*/\\1\\2/g" \
    | awk '{print $NF}' \
    | sort -u > "${prefix}.apt" || true

  printf '%s\n' "${simulation_output}" \
    | grep "pip3 install" \
    | awk '{print $NF}' \
    | sort -u > "${prefix}.pip" || true
}

write_rosdep_manifests /tmp/build-deps
write_rosdep_manifests /tmp/runtime-deps exec

apt-get update
xargs -r apt-get install -y --no-install-recommends -q < /tmp/build-deps.apt
xargs -r env PIP_BREAK_SYSTEM_PACKAGES=1 python3 -m pip install < /tmp/build-deps.pip
apt-get install -y --no-install-recommends \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer1.0-dev \
  pkg-config
rm -rf /var/lib/apt/lists/*
EOF

COPY . src/livekit_ros2_bridge

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && cmake_defs=() \
 && if [[ -n "${LIVEKIT_SDK_DISTRO}" ]]; then cmake_defs+=("-DLIVEKIT_SDK_DISTRO=${LIVEKIT_SDK_DISTRO}"); fi \
 && if [[ -n "${LIVEKIT_SDK_URL_OVERRIDE}" ]]; then cmake_defs+=("-DLIVEKIT_SDK_URL_OVERRIDE=${LIVEKIT_SDK_URL_OVERRIDE}"); fi \
 && if [[ -n "${LIVEKIT_SDK_SHA256_OVERRIDE}" ]]; then cmake_defs+=("-DLIVEKIT_SDK_SHA256_OVERRIDE=${LIVEKIT_SDK_SHA256_OVERRIDE}"); fi \
 && cmake_args=() \
 && if ((${#cmake_defs[@]})); then cmake_args+=(--cmake-args "${cmake_defs[@]}"); fi \
 && colcon build \
      --merge-install \
      --packages-up-to livekit_ros2_bridge \
      "${cmake_args[@]}"

FROM ${ROS_RUNTIME_IMAGE} AS runtime

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

WORKDIR /opt/livekit_ros2_bridge

RUN mkdir -p /config

COPY --from=builder /tmp/runtime-deps.apt /tmp/runtime-deps.apt
COPY --from=builder /tmp/runtime-deps.pip /tmp/runtime-deps.pip

RUN bash <<'EOF'
set -euxo pipefail

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

apt-get update
xargs -r apt-get install -y --no-install-recommends -q < /tmp/runtime-deps.apt
xargs -r env PIP_BREAK_SYSTEM_PACKAGES=1 python3 -m pip install < /tmp/runtime-deps.pip
apt-get install -y --no-install-recommends ca-certificates
rm -rf /var/lib/apt/lists/*
EOF

COPY --from=builder /opt/ros_ws/install /opt/livekit_ros2_bridge/install
COPY --from=gstreamer-publisher-source /usr/local/bin/gstreamer-publisher /usr/local/bin/gstreamer-publisher

RUN bash <<'EOF'
cat > /opt/livekit_ros2_bridge/runtime-entrypoint.sh <<'SCRIPT'
#!/bin/bash
set -euo pipefail

: "${ROS_DISTRO:?ROS_DISTRO must be set}"

restore_nounset() {
  if [[ "${1}" == "1" ]]; then
    set -u
  fi
}

source_bridge_env() {
  local install_prefix="${1:-}"
  local nounset_was_enabled=0

  case $- in
    *u*) nounset_was_enabled=1 ;;
  esac

  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ -f /opt/polymath/setup.bash ]]; then
    # shellcheck disable=SC1091
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
}

if (($#)); then
  source_bridge_env /opt/livekit_ros2_bridge/install
  exec "$@"
fi

params_file="${LIVEKIT_BRIDGE_PARAMS_FILE:-/config/livekit_bridge.params.yaml}"
if [[ ! -r "${params_file}" ]]; then
  echo "[livekit-bridge-launch] ERROR: params file '${params_file}' is not readable; mount a local params file or override the command" >&2
  exit 1
fi

source_bridge_env /opt/livekit_ros2_bridge/install

exec /opt/livekit_ros2_bridge/install/lib/livekit_ros2_bridge/livekit_ros2_bridge_node \
  --ros-args \
  --params-file "${params_file}"
SCRIPT

chmod +x /opt/livekit_ros2_bridge/runtime-entrypoint.sh
EOF

ENTRYPOINT ["/opt/livekit_ros2_bridge/runtime-entrypoint.sh"]
