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

### 3) Create a params file

```bash
cp "$(ros2 pkg prefix --share livekit_ros2_bridge)/config/livekit_bridge.params.yaml" \
  /path/to/livekit_bridge.params.yaml
```

Edit `/path/to/livekit_bridge.params.yaml` and set at least:

```yaml
livekit_bridge:
  ros__parameters:
    livekit.url: "wss://your-livekit.example"
    livekit.room: "robot-room"
    access.static.subscribe.allow: ["/user/*"]
```

You also need auth:

- `livekit.token`, or
- `livekit.api_key` + `livekit.api_secret`

### 4) Run

If you open a new terminal, run `source /opt/ros/<your-distro>/setup.bash` and `source ~/ros2_ws/install/setup.bash` first.

```bash
export LIVEKIT_API_KEY=your-livekit-api-key
export LIVEKIT_API_SECRET=your-livekit-api-secret

ros2 run livekit_ros2_bridge livekit_bridge --ros-args \
  --params-file /path/to/livekit_bridge.params.yaml \
  -p livekit.api_key:="$LIVEKIT_API_KEY" \
  -p livekit.api_secret:="$LIVEKIT_API_SECRET"
```

If no allowlist is set, startup will fail in default-deny mode.

## Configuration

Required on startup:

- `livekit.url`
- `livekit.room`
- `livekit.token` or (`livekit.api_key` + `livekit.api_secret`)
- At least one allowlist entry across publish/subscribe/service

For advanced parameters, see `livekit_ros2_bridge/parameters.yaml`.

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
