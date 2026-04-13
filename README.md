# livekit_ros2_bridge

`livekit_ros2_bridge` is a ROS 2 package that joins a LiveKit room as a ROS-aware participant. It gives LiveKit clients a small, policy-controlled way to discover ROS resources, call ROS services, request topic or video subscriptions, fetch interface definitions, and publish small ROS topic messages into ROS 2.

## When to use it

Use this package when:

- your browser, mobile, or backend client already speaks LiveKit
- you want controlled ROS access instead of exposing the ROS graph directly
- you want ROS interactions and robot video in the same LiveKit room
- you can describe allowed resources with static allow and deny rules

## What it exposes

| Surface | Name | Purpose |
| --- | --- | --- |
| Control topic | `ros.topics.publish` | Best-effort ROS topic publication |
| Control topic | `ros.subscriptions.heartbeat` | Request and renew topic or video subscriptions |
| Control topic | `ros.subscriptions.status` | Receive per-subscription status |
| RPC | `ros.services.call` | Call an authorized ROS service |
| RPC | `ros.services.list` | List authorized ROS services |
| RPC | `ros.topics.list` | List authorized ROS topics |
| RPC | `ros.interfaces.get` | Fetch ROS interface definitions |

The core mental model is:

- clients join the same LiveKit room as the bridge
- discovery and request-response work happen over RPC
- subscriptions stay alive only while clients keep sending heartbeats
- allowed ROS topics are delivered either as CDR data tracks or as video, depending on interface type
- access control is name-based and default-deny

The full contract lives in [docs/protocol.md](./docs/protocol.md) and [docs/subscriptions.md](./docs/subscriptions.md).

## Before you start

You need:

- a ROS 2 workspace where you can build and source this package
- a reachable LiveKit deployment
- a pre-minted `livekit.token`
- at least one allowed topic or service in `access.rules.*.allow`

## Build the package

If you have not built the package yet, run this from the workspace root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to livekit_ros2_bridge
source install/setup.bash
```

If you use this repository's dev-container workflow, `just build` runs the package build there.

## Quickstart

1. Copy the example parameters file.

   ```bash
   cp livekit_bridge.params.example.yaml livekit_bridge.params.yaml
   ```

2. Edit `livekit_bridge.params.yaml` and set the connection, token, and at least one allow rule.

   ```yaml
   livekit_ros2_bridge:
     ros__parameters:
       livekit.url: "wss://your-livekit.example"
       livekit.room: "robot-room"

       livekit.token: ""

       access.rules.subscribe.allow: ["/camera/*"]
       access.rules.publish.allow: ["/cmd_vel"]
       access.rules.service.allow: ["/example/service"]
   ```

   For the full parameter model, read [docs/runtime-configuration.md](./docs/runtime-configuration.md). For rule matching and deny precedence, read [docs/access-control.md](./docs/access-control.md).

   ROS topic subscriptions resolve subscriber `reliability` and `durability` from visible publishers when the bridge creates the subscription. Use `subscription_qos_overrides_ids` together with `subscription.qos_overrides.*` when you need to pin those fields for a topic pattern.

3. If you want video, add root-level `video_topic_rule_ids` and `video_configured_source_ids` plus matching `video.topic_rules.*` and `video.configured_sources.*` entries in the same file. Topic rules match ROS image topics by required `pattern` and optional `transform`. Configured sources define non-ROS video inputs with required `source` and optional `transform`; the configured id becomes the trimmed `configured_source` name clients request. For example, `video.configured_sources.front_rtsp.source: ...` is requested as `{"kind":"configured_source","name":"front_rtsp"}`. Slash and colon variants stay distinct. Use `video.publish.*` for default LiveKit video publish options such as codec, bitrate, framerate, and simulcast, then override any subset per entry with `video.topic_rules.<id>.publish.*` or `video.configured_sources.<id>.publish.*`. The full video model is in [docs/video-sources.md](./docs/video-sources.md).

   This matters for camera topics coming from bridges like `ros_gz_bridge`, which often publish `RELIABLE`. If the publisher is not visible yet when the subscription is created, add a `subscription_qos_overrides_ids` entry and matching `subscription.qos_overrides.*` parameters for that topic pattern.

4. Run the node.

   ```bash
   ros2 run livekit_ros2_bridge livekit_ros2_bridge_node --ros-args \
     --params-file $(pwd)/livekit_bridge.params.yaml
   ```

   Wait for `event=runtime_ready`.

5. Connect a LiveKit client to the same room and use the resources you allowed.

For a fast first success, allow one service or one small topic path first. Add more rules and video sources after that works.

If startup or connection fails, start with [startup and connection issues](./docs/troubleshooting.md#startup-and-connection-issues).

## Current scope

Supported today:

- ROS service calls
- topic and service discovery
- topic subscriptions
- video subscriptions
- small ROS topic publications into ROS 2
- configured video sources

Not supported today:

- ROS actions
- ROS parameter get and set
- full audio support
- large topic publish payloads

## Where to go next

- [docs/README.md](./docs/README.md) for the full documentation map
- [docs/integration.md](./docs/integration.md) for the client integration flow
- [docs/protocol.md](./docs/protocol.md) for the LiveKit-facing contract
- [docs/subscriptions.md](./docs/subscriptions.md) for heartbeat, status, lease, and replay behavior
- [docs/runtime-configuration.md](./docs/runtime-configuration.md) for auth, access, and video configuration
- [docs/runtime-architecture.md](./docs/runtime-architecture.md) for runtime ownership, threading, and reconnect behavior

## License

Apache License 2.0
