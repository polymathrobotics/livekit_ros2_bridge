variable "DEFAULT_PLATFORM" {
  default = "linux/amd64"
}

variable "GSTREAMER_PUBLISHER_REF_JAMMY" {
  default = "8825fee6f40ff51f2cf9347892f6fbc08eeb1f2e"
}

variable "GSTREAMER_PUBLISHER_REF_NOBLE" {
  default = "407891dbdca2ad3113270fbeb350ab9f47615917"
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

group "dev-all" {
  targets = ["dev-humble", "dev-jazzy", "dev-kilted", "dev-rolling"]
}

group "runtime-all" {
  targets = ["runtime-humble", "runtime-jazzy", "runtime-kilted", "runtime-rolling"]
}

group "all" {
  targets = [
    "dev-humble",
    "dev-jazzy",
    "dev-kilted",
    "dev-rolling",
    "runtime-humble",
    "runtime-jazzy",
    "runtime-kilted",
    "runtime-rolling",
  ]
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
  inherits = ["_dev-common"]
  tags = ["livekit_ros2_bridge-builder:humble"]
  args = {
    GSTREAMER_PUBLISHER_IMAGE = GSTREAMER_PUBLISHER_IMAGE_JAMMY
    LIVEKIT_SDK_DISTRO = "jammy"
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:humble-builder-ubuntu"
    ROS_DISTRO = "humble"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:humble-ready-ubuntu"
  }
}

target "dev-jazzy" {
  inherits = ["_dev-common"]
  tags = ["livekit_ros2_bridge-builder:jazzy"]
  args = {
    GSTREAMER_PUBLISHER_IMAGE = GSTREAMER_PUBLISHER_IMAGE_NOBLE
    LIVEKIT_SDK_DISTRO = "noble"
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:jazzy-builder-ubuntu"
    ROS_DISTRO = "jazzy"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:jazzy-ready-ubuntu"
  }
}

target "dev-kilted" {
  inherits = ["_dev-common"]
  tags = ["livekit_ros2_bridge-builder:kilted"]
  args = {
    GSTREAMER_PUBLISHER_IMAGE = GSTREAMER_PUBLISHER_IMAGE_NOBLE
    LIVEKIT_SDK_DISTRO = "noble"
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:kilted-builder-ubuntu"
    ROS_DISTRO = "kilted"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:kilted-ready-ubuntu"
  }
}

target "dev-rolling" {
  inherits = ["_dev-common"]
  tags = ["livekit_ros2_bridge-builder:rolling"]
  args = {
    GSTREAMER_PUBLISHER_IMAGE = GSTREAMER_PUBLISHER_IMAGE_NOBLE
    LIVEKIT_SDK_DISTRO = "noble"
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:rolling-builder-ubuntu"
    ROS_DISTRO = "rolling"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:rolling-ready-ubuntu"
  }
}

target "runtime-humble" {
  inherits = ["_runtime-common"]
  tags = ["livekit_ros2_bridge:humble"]
  args = {
    GSTREAMER_PUBLISHER_IMAGE = GSTREAMER_PUBLISHER_IMAGE_JAMMY
    LIVEKIT_SDK_DISTRO = "jammy"
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:humble-builder-ubuntu"
    ROS_DISTRO = "humble"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:humble-ready-ubuntu"
  }
}

target "runtime-jazzy" {
  inherits = ["_runtime-common"]
  tags = ["livekit_ros2_bridge:jazzy"]
  args = {
    GSTREAMER_PUBLISHER_IMAGE = GSTREAMER_PUBLISHER_IMAGE_NOBLE
    LIVEKIT_SDK_DISTRO = "noble"
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:jazzy-builder-ubuntu"
    ROS_DISTRO = "jazzy"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:jazzy-ready-ubuntu"
  }
}

target "runtime-kilted" {
  inherits = ["_runtime-common"]
  tags = ["livekit_ros2_bridge:kilted"]
  args = {
    GSTREAMER_PUBLISHER_IMAGE = GSTREAMER_PUBLISHER_IMAGE_NOBLE
    LIVEKIT_SDK_DISTRO = "noble"
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:kilted-builder-ubuntu"
    ROS_DISTRO = "kilted"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:kilted-ready-ubuntu"
  }
}

target "runtime-rolling" {
  inherits = ["_runtime-common"]
  tags = ["livekit_ros2_bridge:rolling"]
  args = {
    GSTREAMER_PUBLISHER_IMAGE = GSTREAMER_PUBLISHER_IMAGE_NOBLE
    LIVEKIT_SDK_DISTRO = "noble"
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:rolling-builder-ubuntu"
    ROS_DISTRO = "rolling"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:rolling-ready-ubuntu"
  }
}
