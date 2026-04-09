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

COPY docker/scripts/generate-rosdep-install-script.sh /usr/local/bin/generate-rosdep-install-script
COPY docker/scripts/configure-parameter-library-apt-pin.sh /usr/local/bin/configure-parameter-library-apt-pin

RUN chmod +x \
      /usr/local/bin/generate-rosdep-install-script \
      /usr/local/bin/configure-parameter-library-apt-pin \
 && rosdep update --rosdistro "${ROS_DISTRO}"

FROM ros-builder-base AS dev

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO

ENV CCACHE_DIR=/ccache

WORKDIR /opt/dev-bootstrap

COPY package.xml /opt/dev-bootstrap/src/livekit_ros2_bridge/package.xml

# Jazzy, Kilted, and Rolling still lag on packages.ros.org main for the
# generate_parameter_library family. Add ros2-testing, but pin only those
# packages so the rest of ROS continues to come from the stable main repo.
RUN /usr/local/bin/configure-parameter-library-apt-pin "${ROS_DISTRO}" \
 && generate-rosdep-install-script /tmp/install-dev-deps.sh /opt/dev-bootstrap/src \
 && apt-get update \
 && /tmp/install-dev-deps.sh \
 && apt-get install -y --no-install-recommends \
      ca-certificates \
      ccache \
      git \
      libgstreamer-plugins-base1.0-dev \
      libgstreamer1.0-dev \
      pkg-config \
      ros-${ROS_DISTRO}-ros2cli \
      ros-${ROS_DISTRO}-ros2cli-common-extensions \
 && rm -rf /var/lib/apt/lists/*

COPY --from=gstreamer-publisher-source /usr/local/bin/gstreamer-publisher /usr/local/bin/gstreamer-publisher
COPY docker/scripts/source-bridge-env.sh /usr/local/bin/livekit-bridge-source-env
COPY docker/scripts/source-workspace.sh /usr/local/bin/livekit-bridge-dev-env
COPY docker/scripts/dev-entrypoint.sh /usr/local/bin/livekit-bridge-dev-entrypoint

RUN chmod +x \
      /usr/local/bin/gstreamer-publisher \
      /usr/local/bin/livekit-bridge-source-env \
      /usr/local/bin/livekit-bridge-dev-env \
      /usr/local/bin/livekit-bridge-dev-entrypoint

WORKDIR /workspace

ENTRYPOINT ["/usr/local/bin/livekit-bridge-dev-entrypoint"]
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
RUN /usr/local/bin/configure-parameter-library-apt-pin "${ROS_DISTRO}" \
 && generate-rosdep-install-script /tmp/install-build-deps.sh src \
 && DEPENDENCY_TYPES=exec generate-rosdep-install-script /tmp/install-runtime-deps.sh src \
 && apt-get update \
 && /tmp/install-build-deps.sh \
 && apt-get install -y --no-install-recommends \
      libgstreamer-plugins-base1.0-dev \
      libgstreamer1.0-dev \
      pkg-config \
 && rm -rf /var/lib/apt/lists/*

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
ENV GST_PLUGIN_PATH=/opt/livekit_ros2_bridge/install/lib/livekit_ros2_bridge

WORKDIR /opt/livekit_ros2_bridge

COPY docker/scripts/source-bridge-env.sh /usr/local/bin/livekit-bridge-source-env
COPY docker/scripts/runtime-entrypoint.sh /usr/local/bin/livekit-bridge-entrypoint
COPY docker/scripts/runtime-launch-bridge.sh /usr/local/bin/livekit-bridge-launch
COPY docker/scripts/runtime-smoke.sh /usr/local/bin/livekit-bridge-runtime-smoke

RUN chmod +x \
      /usr/local/bin/livekit-bridge-source-env \
      /usr/local/bin/livekit-bridge-entrypoint \
      /usr/local/bin/livekit-bridge-launch \
      /usr/local/bin/livekit-bridge-runtime-smoke

COPY --from=builder /tmp/install-runtime-deps.sh /tmp/install-runtime-deps.sh

RUN apt-get update \
 && /tmp/install-runtime-deps.sh \
 && apt-get install -y --no-install-recommends ca-certificates \
 && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/ros_ws/install /opt/livekit_ros2_bridge/install
COPY --from=gstreamer-publisher-source /usr/local/bin/gstreamer-publisher /usr/local/bin/gstreamer-publisher

RUN /usr/local/bin/livekit-bridge-runtime-smoke

ENTRYPOINT ["/usr/local/bin/livekit-bridge-entrypoint"]
CMD ["bash"]
