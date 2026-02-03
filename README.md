# LiveKit ROS2 Bridge

Generic bridge for connecting ROS2 pub/sub into LiveKit.

## Installation

### From Source

```bash
cd ~/ros2_ws/src
git clone https://github.com/polymathrobotics/livekit_ros2_bridge.git
cd ~/ros2_ws
rosdep install -r --from-paths src -i -y
colcon build --packages-select livekit_ros2_bridge
```

## Usage

### CLI

```bash
ros2 run livekit_ros2_bridge livekit_bridge
```

For help:

```bash
ros2 run livekit_ros2_bridge livekit_bridge --help
```

## Development

### Building

```bash
colcon build --packages-select livekit_ros2_bridge --cmake-args "-DCMAKE_EXPORT_COMPILE_COMMANDS=On"
```

### Testing

```bash
colcon test --packages-select livekit_ros2_bridge
colcon test-result --verbose
```

### Activating Code Standard Hooks

[Pre-commit](https://pre-commit.com) hooks are provided to maintain code standards for this repository.

1. If you do not have pre-commit installed, run `python3 -m pip install pre-commit`
1. For preexisting repositories, you must run `pre-commit install` in that repository
1. You can automatically install pre-commit for newly cloned repositories by running

    ```bash
    git config --global init.templateDir ~/.git-template
    pre-commit init-templatedir ~/.git-template
    ```

Now all git commits will be automatically gated by the configured checks.

## License

Apache License 2.0
