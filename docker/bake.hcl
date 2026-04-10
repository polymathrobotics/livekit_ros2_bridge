variable "DEFAULT_PLATFORM" {
  default = "linux/amd64"
}

variable "GSTREAMER_PUBLISHER_IMAGE_JAMMY" {
  default = "docker.io/polymathrobotics/gstreamer-publisher:jammy-8825fee6f40ff51f2cf9347892f6fbc08eeb1f2e"
}

variable "GSTREAMER_PUBLISHER_IMAGE_NOBLE" {
  default = "docker.io/polymathrobotics/gstreamer-publisher:noble-407891dbdca2ad3113270fbeb350ab9f47615917"
}

group "default" {
  targets = ["dev-humble"]
}

group "dev-defaults" {
  targets = ["dev-humble", "dev-rolling"]
}

group "dev-all" {
  targets = ["dev-humble", "dev-jazzy", "dev-kilted", "dev-rolling"]
}

group "runtime-all" {
  targets = ["runtime-humble", "runtime-jazzy", "runtime-kilted", "runtime-rolling"]
}

group "ci-docker" {
  targets = ["runtime-all", "dev-defaults"]
}

group "all" {
  targets = ["dev-all", "runtime-all"]
}

target "_common" {
  context = "."
  dockerfile = "docker/Dockerfile"
  platforms = [DEFAULT_PLATFORM]
}

target "_dev-common" {
  inherits = ["_common"]
  target = "dev"
}

target "_runtime-common" {
  inherits = ["_common"]
  target = "runtime"
}

target "_humble-common" {
  args = {
    GSTREAMER_PUBLISHER_IMAGE = GSTREAMER_PUBLISHER_IMAGE_JAMMY
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:humble-builder-ubuntu"
    ROS_DISTRO = "humble"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:humble-ready-ubuntu"
  }
}

target "_jazzy-common" {
  args = {
    GSTREAMER_PUBLISHER_IMAGE = GSTREAMER_PUBLISHER_IMAGE_NOBLE
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:jazzy-builder-ubuntu"
    ROS_DISTRO = "jazzy"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:jazzy-ready-ubuntu"
  }
}

target "_kilted-common" {
  args = {
    GSTREAMER_PUBLISHER_IMAGE = GSTREAMER_PUBLISHER_IMAGE_NOBLE
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:kilted-builder-ubuntu"
    ROS_DISTRO = "kilted"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:kilted-ready-ubuntu"
  }
}

target "_rolling-common" {
  args = {
    GSTREAMER_PUBLISHER_IMAGE = GSTREAMER_PUBLISHER_IMAGE_NOBLE
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:rolling-builder-ubuntu"
    ROS_DISTRO = "rolling"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:rolling-ready-ubuntu"
  }
}

# Humble uses the Jammy-compatible gstreamer-publisher image, while newer ROS
# distros use the Noble-compatible one.
#
# There are two separate reasons the Humble/Jammy image needs an older upstream
# revision:
# 1. It now requires Go 1.24.4 or newer.
# 2. Even with a new enough Go toolchain, the build still fails on Jammy because
#    the go-gst dependency calls gst_debug_message_get_id, and Jammy's GStreamer
#    packages do not provide that symbol.
#
# The older revision still builds correctly for Humble, so the bridge selects
# the Jammy image there for compatibility rather than because Humble itself
# requires an older Go toolchain.
target "dev-humble" {
  inherits = ["_dev-common", "_humble-common"]
  tags = ["livekit_ros2_bridge-dev:humble"]
}

target "dev-jazzy" {
  inherits = ["_dev-common", "_jazzy-common"]
  tags = ["livekit_ros2_bridge-dev:jazzy"]
}

target "dev-kilted" {
  inherits = ["_dev-common", "_kilted-common"]
  tags = ["livekit_ros2_bridge-dev:kilted"]
}

target "dev-rolling" {
  inherits = ["_dev-common", "_rolling-common"]
  tags = ["livekit_ros2_bridge-dev:rolling"]
}

target "runtime-humble" {
  inherits = ["_runtime-common", "_humble-common"]
  tags = ["livekit_ros2_bridge:humble"]
}

target "runtime-jazzy" {
  inherits = ["_runtime-common", "_jazzy-common"]
  tags = ["livekit_ros2_bridge:jazzy"]
}

target "runtime-kilted" {
  inherits = ["_runtime-common", "_kilted-common"]
  tags = ["livekit_ros2_bridge:kilted"]
}

target "runtime-rolling" {
  inherits = ["_runtime-common", "_rolling-common"]
  tags = ["livekit_ros2_bridge:rolling"]
}
