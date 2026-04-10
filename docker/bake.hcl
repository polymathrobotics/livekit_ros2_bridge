variable "DEFAULT_PLATFORM" {
  default = "linux/amd64"
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
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:humble-builder-ubuntu"
    ROS_DISTRO = "humble"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:humble-ready-ubuntu"
  }
}

target "_jazzy-common" {
  args = {
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:jazzy-builder-ubuntu"
    ROS_DISTRO = "jazzy"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:jazzy-ready-ubuntu"
  }
}

target "_kilted-common" {
  args = {
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:kilted-builder-ubuntu"
    ROS_DISTRO = "kilted"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:kilted-ready-ubuntu"
  }
}

target "_rolling-common" {
  args = {
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:rolling-builder-ubuntu"
    ROS_DISTRO = "rolling"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:rolling-ready-ubuntu"
  }
}
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
