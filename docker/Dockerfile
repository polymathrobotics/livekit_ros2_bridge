# syntax=docker/dockerfile:1

ARG ROS_DISTRO=humble
ARG ROS_BUILDER_IMAGE=polymathrobotics/ros:${ROS_DISTRO}-builder-ubuntu
ARG ROS_RUNTIME_IMAGE=polymathrobotics/ros:${ROS_DISTRO}-ready-ubuntu

FROM ${ROS_BUILDER_IMAGE} AS toolchain-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

RUN rosdep update --rosdistro "${ROS_DISTRO}"

FROM toolchain-base AS deps-scripts

ARG SKIP_KEYS=""

WORKDIR /opt/rosdep-workspace

COPY package.xml src/livekit_ros2_bridge/package.xml
COPY docker/gather-rosdeps.sh /usr/local/bin/gather-rosdeps

RUN chmod +x /usr/local/bin/gather-rosdeps \
     && export SKIP_KEYS="${SKIP_KEYS}" \
     && gather-rosdeps /tmp/rosdep-scripts/install-dev-rosdeps.sh src/livekit_ros2_bridge \
     && ROSDEP_DEPENDENCY_TYPES="buildtool buildtool_export build build_export exec" \
     gather-rosdeps /tmp/rosdep-scripts/install-build-rosdeps.sh src/livekit_ros2_bridge \
     && ROSDEP_DEPENDENCY_TYPES="exec" \
     gather-rosdeps /tmp/rosdep-scripts/install-runtime-rosdeps.sh src/livekit_ros2_bridge

FROM toolchain-base AS runtime-builder

WORKDIR /opt/ros_ws

COPY --from=deps-scripts /tmp/rosdep-scripts/install-build-rosdeps.sh /tmp/install-build-rosdeps.sh

RUN apt-get update \
     && /tmp/install-build-rosdeps.sh \
     && rm -rf /var/lib/apt/lists/*

COPY CMakeLists.txt package.xml parameters.yaml src/livekit_ros2_bridge/
COPY cmake src/livekit_ros2_bridge/cmake
COPY include src/livekit_ros2_bridge/include
COPY src src/livekit_ros2_bridge/src

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
     && colcon build \
     --merge-install \
     --packages-up-to livekit_ros2_bridge \
     --cmake-args -DBUILD_TESTING=OFF

FROM toolchain-base AS dev

ARG ROS_DISTRO

ENV CCACHE_DIR=/ccache

COPY --from=deps-scripts /tmp/rosdep-scripts/install-dev-rosdeps.sh /tmp/install-dev-rosdeps.sh
COPY --from=runtime-builder /opt/ros_ws/install /opt/livekit_ros2_bridge/install

RUN apt-get update \
     && /tmp/install-dev-rosdeps.sh \
     && apt-get install -y --no-install-recommends \
     ros-${ROS_DISTRO}-ros2cli \
     ros-${ROS_DISTRO}-ros2cli-common-extensions \
     && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

CMD ["sleep", "infinity"]

FROM ${ROS_RUNTIME_IMAGE} AS runtime

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

WORKDIR /opt/livekit_ros2_bridge

COPY docker/runtime-entrypoint.sh /opt/livekit_ros2_bridge/runtime-entrypoint.sh
COPY --from=deps-scripts /tmp/rosdep-scripts/install-runtime-rosdeps.sh /tmp/install-runtime-rosdeps.sh

# ca-certificates is needed for LiveKit connections
RUN mkdir -p /config \
     && chmod +x /opt/livekit_ros2_bridge/runtime-entrypoint.sh /tmp/install-runtime-rosdeps.sh \
     && apt-get update \
     && apt-get install -y --no-install-recommends ca-certificates \
     && /tmp/install-runtime-rosdeps.sh \
     && rm -rf /var/lib/apt/lists/*

COPY --from=runtime-builder /opt/ros_ws/install /opt/livekit_ros2_bridge/install

ENTRYPOINT ["/opt/livekit_ros2_bridge/runtime-entrypoint.sh"]
