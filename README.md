# LiveKit ROS2 Bridge

`livekit_ros2_bridge` connects a LiveKit room to ROS 2 topics and services.
It uses JSON messages over LiveKit for control and topic data.
Each bridge instance connects to one LiveKit room.

How the bridge communicates over LiveKit:

- 1 inbound data topic (`ros.topic.publish`)
- 1 outbound data topic (`ros.topic.messages`)
- 3 control RPCs (`ros.topic.subscribe`, `ros.topic.unsubscribe`, `ros.service.call`)

| Plane | Initiator | LiveKit interface | Bridge action | Result |
| --- | --- | --- | --- | --- |
| Data in | LiveKit | Topic: `ros.topic.publish` | Publish a ROS topic message | ROS topic receives message |
| Control | LiveKit | RPC: `ros.topic.subscribe` | Create a ROS subscription | ROS topic messages start on `ros.topic.messages` |
| Control | LiveKit | RPC: `ros.topic.unsubscribe` | Remove a ROS subscription | ROS topic messages stop on `ros.topic.messages` |
| Control | LiveKit | RPC: `ros.service.call` | Call a ROS service | RPC returns service response |
| Data out | Bridge | Topic: `ros.topic.messages` | Send subscribed ROS topic messages | LiveKit receives ROS topic message stream |

## Quick start

### 1) Requirements

- ROS 2 (tested in CI: Humble, Jazzy, Kilted, Rolling)
- A LiveKit deployment (LiveKit Cloud or self-hosted)

### 2) Install from source

In a ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/polymathrobotics/livekit_ros2_bridge.git
cd ~/ros2_ws
rosdep install -r --from-paths src -i -y
colcon build --packages-select livekit_ros2_bridge
```

Install the LiveKit Python SDK in the same environment where the node will run:

```bash
python3 -m pip install livekit==1.0.23 livekit-api==1.1.0 livekit-protocol==1.1.2
```

### 3) Set LiveKit connection and auth

```bash
export LIVEKIT_URL=...
export LIVEKIT_ROOM=...

# Set either:
export LIVEKIT_TOKEN=...

# Or:
# export LIVEKIT_API_KEY=...
# export LIVEKIT_API_SECRET=...
```

### 4) Create an overlay params file

Copy the packaged template:

```bash
cp $(ros2 pkg prefix --share livekit_ros2_bridge)/config/livekit_bridge.overlay.yaml \
  /path/to/your_livekit_bridge.overlay.yaml
```

Edit `/path/to/your_livekit_bridge.overlay.yaml` and start with this:

This bridge is **default-deny**. If all allowlists are empty, startup is rejected.
That is on purpose, so nothing is exposed unless you allow it.

```yaml
livekit_bridge:
  ros__parameters:
    access.static.publish.allow: ["/example/foo"]
    access.static.subscribe.allow: ["/example/*"]
    access.static.service.allow: ["/example/service/a", "/example/service/b"]
```

Quick notes:

- The launch file loads `params_file` first and `overlay_params_file` second (overrides)
- YAML substitutions are enabled, so `$(env ...)` works in params files
- The packaged `config/livekit_bridge.yaml` is a template; keep runtime-specific changes in your overlay file

### 5) Launch

```bash
ros2 launch livekit_ros2_bridge livekit_bridge.launch.yaml \
  overlay_params_file:=/path/to/your_livekit_bridge.overlay.yaml
```

Troubleshooting guide: [docs/troubleshooting.md](docs/troubleshooting.md)

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
