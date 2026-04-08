variable "DEFAULT_PLATFORM" {
  default = "linux/amd64"
}

variable "PARAMETER_LIBRARY_OVERLAY_REF" {
  default = "0.7.1"
}

variable "GSTREAMER_PUBLISHER_REF_JAMMY" {
  default = "8825fee6f40ff51f2cf9347892f6fbc08eeb1f2e"
}

variable "GSTREAMER_PUBLISHER_REF_NOBLE" {
  default = "407891dbdca2ad3113270fbeb350ab9f47615917"
}

variable "GSTREAMER_PUBLISHER_IMAGE_REPOSITORY" {
  default = "ghcr.io/polymathrobotics/livekit_ros2_bridge-gstreamer-publisher"
}

variable "GSTREAMER_PUBLISHER_IMAGE_JAMMY" {
  default = "ghcr.io/polymathrobotics/livekit_ros2_bridge-gstreamer-publisher:jammy-8825fee6f40ff51f2cf9347892f6fbc08eeb1f2e"
}

variable "GSTREAMER_PUBLISHER_IMAGE_NOBLE" {
  default = "ghcr.io/polymathrobotics/livekit_ros2_bridge-gstreamer-publisher:noble-407891dbdca2ad3113270fbeb350ab9f47615917"
}

group "default" {
  targets = ["dev-humble"]
}

group "publisher-all" {
  targets = ["publisher-jammy", "publisher-noble"]
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

target "_publisher-common" {
  inherits = ["_common"]
  target = "gstreamer-publisher"
}

# Humble uses Ubuntu Jammy, and Jammy cannot build our newer
# gstreamer-publisher revision.
#
# There are two separate problems with that newer revision:
# 1. It now requires Go 1.24.4 or newer.
# 2. Even with a new enough Go toolchain, the build still fails on Jammy because
#    the go-gst dependency calls gst_debug_message_get_id, and Jammy's GStreamer
#    packages do not provide that symbol.
#
# The older gstreamer-publisher revision below still builds correctly for Humble,
# including with Go 1.24. So Humble is pinned to an older publisher revision
# because of Jammy compatibility, not because Humble needs an older Go version.
target "dev-humble" {
  inherits = ["_dev-common"]
  tags = ["livekit_ros2_bridge-builder:humble"]
  args = {
    ENABLE_PARAMETER_LIBRARY_OVERLAY = "1"
    GOLANG_IMAGE = "polymathrobotics/golang:1.24-noble"
    GSTREAMER_PUBLISHER_REF = GSTREAMER_PUBLISHER_REF_JAMMY
    LIVEKIT_SDK_DISTRO = "jammy"
    PARAMETER_LIBRARY_OVERLAY_REF = PARAMETER_LIBRARY_OVERLAY_REF
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:humble-builder-ubuntu"
    ROS_DISTRO = "humble"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:humble-ready-ubuntu"
  }
}

target "dev-jazzy" {
  inherits = ["_dev-common"]
  tags = ["livekit_ros2_bridge-builder:jazzy"]
  args = {
    ENABLE_PARAMETER_LIBRARY_OVERLAY = "1"
    GOLANG_IMAGE = "polymathrobotics/golang:1.24-noble"
    GSTREAMER_PUBLISHER_REF = GSTREAMER_PUBLISHER_REF_NOBLE
    LIVEKIT_SDK_DISTRO = "noble"
    PARAMETER_LIBRARY_OVERLAY_REF = PARAMETER_LIBRARY_OVERLAY_REF
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:jazzy-builder-ubuntu"
    ROS_DISTRO = "jazzy"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:jazzy-ready-ubuntu"
  }
}

target "dev-kilted" {
  inherits = ["_dev-common"]
  tags = ["livekit_ros2_bridge-builder:kilted"]
  args = {
    ENABLE_PARAMETER_LIBRARY_OVERLAY = "1"
    GOLANG_IMAGE = "polymathrobotics/golang:1.24-noble"
    GSTREAMER_PUBLISHER_REF = GSTREAMER_PUBLISHER_REF_NOBLE
    LIVEKIT_SDK_DISTRO = "noble"
    PARAMETER_LIBRARY_OVERLAY_REF = PARAMETER_LIBRARY_OVERLAY_REF
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:kilted-builder-ubuntu"
    ROS_DISTRO = "kilted"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:kilted-ready-ubuntu"
  }
}

target "dev-rolling" {
  inherits = ["_dev-common"]
  tags = ["livekit_ros2_bridge-builder:rolling"]
  args = {
    ENABLE_PARAMETER_LIBRARY_OVERLAY = "1"
    GOLANG_IMAGE = "polymathrobotics/golang:1.24-noble"
    GSTREAMER_PUBLISHER_REF = GSTREAMER_PUBLISHER_REF_NOBLE
    LIVEKIT_SDK_DISTRO = "noble"
    PARAMETER_LIBRARY_OVERLAY_REF = PARAMETER_LIBRARY_OVERLAY_REF
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:rolling-builder-ubuntu"
    ROS_DISTRO = "rolling"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:rolling-ready-ubuntu"
  }
}

target "runtime-humble" {
  inherits = ["_runtime-common"]
  tags = ["livekit_ros2_bridge:humble"]
  args = {
    ENABLE_PARAMETER_LIBRARY_OVERLAY = "0"
    GOLANG_IMAGE = "polymathrobotics/golang:1.24-noble"
    GSTREAMER_PUBLISHER_REF = GSTREAMER_PUBLISHER_REF_JAMMY
    LIVEKIT_SDK_DISTRO = "jammy"
    PARAMETER_LIBRARY_OVERLAY_REF = PARAMETER_LIBRARY_OVERLAY_REF
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:humble-builder-ubuntu"
    ROS_DISTRO = "humble"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:humble-ready-ubuntu"
  }
}

target "runtime-jazzy" {
  inherits = ["_runtime-common"]
  tags = ["livekit_ros2_bridge:jazzy"]
  args = {
    ENABLE_PARAMETER_LIBRARY_OVERLAY = "1"
    GOLANG_IMAGE = "polymathrobotics/golang:1.24-noble"
    GSTREAMER_PUBLISHER_REF = GSTREAMER_PUBLISHER_REF_NOBLE
    LIVEKIT_SDK_DISTRO = "noble"
    PARAMETER_LIBRARY_OVERLAY_REF = PARAMETER_LIBRARY_OVERLAY_REF
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:jazzy-builder-ubuntu"
    ROS_DISTRO = "jazzy"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:jazzy-ready-ubuntu"
  }
}

target "runtime-kilted" {
  inherits = ["_runtime-common"]
  tags = ["livekit_ros2_bridge:kilted"]
  args = {
    ENABLE_PARAMETER_LIBRARY_OVERLAY = "1"
    GOLANG_IMAGE = "polymathrobotics/golang:1.24-noble"
    GSTREAMER_PUBLISHER_REF = GSTREAMER_PUBLISHER_REF_NOBLE
    LIVEKIT_SDK_DISTRO = "noble"
    PARAMETER_LIBRARY_OVERLAY_REF = PARAMETER_LIBRARY_OVERLAY_REF
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:kilted-builder-ubuntu"
    ROS_DISTRO = "kilted"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:kilted-ready-ubuntu"
  }
}

target "runtime-rolling" {
  inherits = ["_runtime-common"]
  tags = ["livekit_ros2_bridge:rolling"]
  args = {
    ENABLE_PARAMETER_LIBRARY_OVERLAY = "1"
    GOLANG_IMAGE = "polymathrobotics/golang:1.24-noble"
    GSTREAMER_PUBLISHER_REF = GSTREAMER_PUBLISHER_REF_NOBLE
    LIVEKIT_SDK_DISTRO = "noble"
    PARAMETER_LIBRARY_OVERLAY_REF = PARAMETER_LIBRARY_OVERLAY_REF
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:rolling-builder-ubuntu"
    ROS_DISTRO = "rolling"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:rolling-ready-ubuntu"
  }
}

target "publisher-jammy" {
  inherits = ["_publisher-common"]
  tags = [
    "ghcr.io/polymathrobotics/livekit_ros2_bridge-gstreamer-publisher:jammy",
    GSTREAMER_PUBLISHER_IMAGE_JAMMY,
  ]
  args = {
    GOLANG_IMAGE = "polymathrobotics/golang:1.24-noble"
    GSTREAMER_PUBLISHER_REF = GSTREAMER_PUBLISHER_REF_JAMMY
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:humble-builder-ubuntu"
    ROS_DISTRO = "humble"
  }
}

target "publisher-noble" {
  inherits = ["_publisher-common"]
  tags = [
    "ghcr.io/polymathrobotics/livekit_ros2_bridge-gstreamer-publisher:noble",
    GSTREAMER_PUBLISHER_IMAGE_NOBLE,
  ]
  args = {
    GOLANG_IMAGE = "polymathrobotics/golang:1.24-noble"
    GSTREAMER_PUBLISHER_REF = GSTREAMER_PUBLISHER_REF_NOBLE
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:rolling-builder-ubuntu"
    ROS_DISTRO = "rolling"
  }
}
