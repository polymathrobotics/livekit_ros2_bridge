# syntax=docker/dockerfile:1

ARG ROS_DISTRO=humble
ARG ROS_BUILDER_IMAGE=polymathrobotics/ros:${ROS_DISTRO}-builder-ubuntu
ARG ROS_RUNTIME_IMAGE=polymathrobotics/ros:${ROS_DISTRO}-ready-ubuntu
ARG GOLANG_IMAGE=polymathrobotics/golang:1.24-noble
ARG GSTREAMER_PUBLISHER_REF=407891dbdca2ad3113270fbeb350ab9f47615917
ARG GSTREAMER_PUBLISHER_SOURCE=gstreamer-publisher
ARG ENABLE_PARAMETER_LIBRARY_OVERLAY=1
ARG PARAMETER_LIBRARY_OVERLAY_REF=0.7.1
ARG LIVEKIT_SDK_DISTRO=
ARG LIVEKIT_SDK_URL_OVERRIDE=
ARG LIVEKIT_SDK_SHA256_OVERRIDE=

FROM ${GOLANG_IMAGE} AS golang-toolchain

FROM ${ROS_BUILDER_IMAGE} AS gstreamer-publisher-builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG GSTREAMER_PUBLISHER_REF

ENV DEBIAN_FRONTEND=noninteractive
ENV GOTOOLCHAIN=local
ENV GOPATH=/go
ENV PATH=/go/bin:/usr/local/go/bin:${PATH}

WORKDIR /tmp

COPY --from=golang-toolchain /usr/local/go /usr/local/go

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      git \
      libgstreamer-plugins-base1.0-dev \
      libgstreamer1.0-dev \
      pkg-config \
 && rm -rf /var/lib/apt/lists/*

RUN git init /tmp/gstreamer-publisher \
 && git -C /tmp/gstreamer-publisher remote add origin https://github.com/livekit/gstreamer-publisher \
 && git -C /tmp/gstreamer-publisher fetch --depth 1 origin "${GSTREAMER_PUBLISHER_REF}" \
 && git -C /tmp/gstreamer-publisher checkout FETCH_HEAD \
 && GOFLAGS='-p=1' go build -C /tmp/gstreamer-publisher -o /usr/local/bin/gstreamer-publisher .

FROM scratch AS gstreamer-publisher

COPY --from=gstreamer-publisher-builder /usr/local/bin/gstreamer-publisher /usr/local/bin/gstreamer-publisher

FROM ${GSTREAMER_PUBLISHER_SOURCE} AS gstreamer-publisher-source

FROM ${ROS_BUILDER_IMAGE} AS ros-builder-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV ROS_DISTRO=${ROS_DISTRO}

COPY docker/scripts/generate-rosdep-install-script.sh /usr/local/bin/generate-rosdep-install-script
COPY docker/scripts/apply-parameter-library-overlay.sh /usr/local/bin/apply-parameter-library-overlay

RUN chmod +x \
      /usr/local/bin/generate-rosdep-install-script \
      /usr/local/bin/apply-parameter-library-overlay \
 && rosdep update --rosdistro "${ROS_DISTRO}"

FROM ros-builder-base AS dev

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO
ARG PARAMETER_LIBRARY_OVERLAY_REF

ENV CCACHE_DIR=/ccache

WORKDIR /opt/dev-bootstrap

COPY package.xml /opt/dev-bootstrap/src/livekit_ros2_bridge/package.xml

# Distro packages still ship generate_parameter_library_py 0.6.x, which mis-generates
# nested map parameters. Overlay 0.7.1 until the ROS packages catch up.
RUN generate-rosdep-install-script /tmp/install-dev-deps.sh /opt/dev-bootstrap/src \
 && apt-get update \
 && /tmp/install-dev-deps.sh \
 && apt-get install -y --no-install-recommends \
      ca-certificates \
      ccache \
      git \
      libgstreamer-plugins-base1.0-dev \
      libgstreamer1.0-dev \
      pkg-config \
      python3-pip \
      ros-${ROS_DISTRO}-ros2cli \
      ros-${ROS_DISTRO}-ros2cli-common-extensions \
 && /usr/local/bin/apply-parameter-library-overlay "${ROS_DISTRO}" "${PARAMETER_LIBRARY_OVERLAY_REF}" \
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
ARG ENABLE_PARAMETER_LIBRARY_OVERLAY
ARG PARAMETER_LIBRARY_OVERLAY_REF
ARG LIVEKIT_SDK_DISTRO
ARG LIVEKIT_SDK_URL_OVERRIDE
ARG LIVEKIT_SDK_SHA256_OVERRIDE

WORKDIR /opt/ros_ws

COPY package.xml src/livekit_ros2_bridge/package.xml

# Distro packages still ship generate_parameter_library_py 0.6.x, which mis-generates
# nested map parameters. Overlay 0.7.1 until the ROS packages catch up.
RUN generate-rosdep-install-script /tmp/install-build-deps.sh src \
 && DEPENDENCY_TYPES=exec generate-rosdep-install-script /tmp/install-runtime-deps.sh src \
 && apt-get update \
 && /tmp/install-build-deps.sh \
 && build_packages=( \
      libgstreamer-plugins-base1.0-dev \
      libgstreamer1.0-dev \
      pkg-config \
    ) \
 && if [[ "${ENABLE_PARAMETER_LIBRARY_OVERLAY}" == "1" ]]; then build_packages+=(python3-pip); fi \
 && apt-get install -y --no-install-recommends "${build_packages[@]}" \
 && if [[ "${ENABLE_PARAMETER_LIBRARY_OVERLAY}" == "1" ]]; then /usr/local/bin/apply-parameter-library-overlay "${ROS_DISTRO}" "${PARAMETER_LIBRARY_OVERLAY_REF}"; fi \
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
