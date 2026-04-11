variable "DEFAULT_PLATFORM" {
  default = "linux/amd64"
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

group "test-all" {
  targets = ["test-humble", "test-jazzy", "test-kilted", "test-rolling"]
}

group "ci-test" {
  targets = ["test-all"]
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

target "_test-common" {
  inherits = ["_common"]
  target = "test"
}

target "_humble-common" {
  args = {
    LIVEKIT_SDK_DISTRO = "jammy"
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:humble-builder-ubuntu"
    ROS_DISTRO = "humble"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:humble-ready-ubuntu"
  }
}

target "_jazzy-common" {
  args = {
    LIVEKIT_SDK_DISTRO = "noble"
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:jazzy-builder-ubuntu"
    ROS_DISTRO = "jazzy"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:jazzy-ready-ubuntu"
  }
}

target "_kilted-common" {
  args = {
    LIVEKIT_SDK_DISTRO = "noble"
    ROS_BUILDER_IMAGE = "polymathrobotics/ros:kilted-builder-ubuntu"
    ROS_DISTRO = "kilted"
    ROS_RUNTIME_IMAGE = "polymathrobotics/ros:kilted-ready-ubuntu"
  }
}

target "_rolling-common" {
  args = {
    LIVEKIT_SDK_DISTRO = "noble"
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

target "test-humble" {
  inherits = ["_test-common", "_humble-common"]
  tags = ["livekit_ros2_bridge-test:humble"]
}

target "runtime-jazzy" {
  inherits = ["_runtime-common", "_jazzy-common"]
  tags = ["livekit_ros2_bridge:jazzy"]
}

target "test-jazzy" {
  inherits = ["_test-common", "_jazzy-common"]
  tags = ["livekit_ros2_bridge-test:jazzy"]
}

target "runtime-kilted" {
  inherits = ["_runtime-common", "_kilted-common"]
  tags = ["livekit_ros2_bridge:kilted"]
}

target "test-kilted" {
  inherits = ["_test-common", "_kilted-common"]
  tags = ["livekit_ros2_bridge-test:kilted"]
}

target "runtime-rolling" {
  inherits = ["_runtime-common", "_rolling-common"]
  tags = ["livekit_ros2_bridge:rolling"]
}

target "test-rolling" {
  inherits = ["_test-common", "_rolling-common"]
  tags = ["livekit_ros2_bridge-test:rolling"]
}
