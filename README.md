# livekit_ros2_bridge

`livekit_ros2_bridge` is a ROS 2 package that joins a LiveKit room as a ROS-aware participant. It gives LiveKit clients a way to request topic or video subscriptions, fetch interface definitions, call ROS services, and publish small ROS topic messages.

The core mental model is:

- clients join the same LiveKit room as the bridge
- clients send a heartbeat that says "this is the full set of subscriptions I still want"
  - the bridge streams the requested subscriptions over LiveKit data and video tracks
  - the bridge stops a track if the heartbeats stop or if a subscription is dropped in later heartbeats
- the bridge responds to heartbeats with a status update that clients use to correlate a subscription with a data or video track.
- access control is done with global allow and deny lists by topic
- discovery and request-response work happen over RPC

## What it exposes

The bridge uses three kinds of LiveKit surfaces:

- RPCs for request-response flows such as listing resources, fetching interface definitions, and calling services
- data-packet topics for ROS publish requests and subscription control-plane messages
- tracks for ongoing delivery of ROS topic data or video

| Surface | Name | Role | Purpose |
| --- | --- | --- | --- |
| Data-Packet Topic | `ros2.topic.pub` | ROS Publish Request | Best-effort ROS topic publication |
| Data-Packet Topic | `lkros.heartbeat` | Control-Plane Request | Request and renew topic or video subscriptions |
| Data-Packet Topic | `lkros.status` | Control-Plane Status | The named LiveKit tracks of what the bridge actually made available after a heartbeat |
| RPC | `ros2.service.call` | Request-Response | Call an authorized ROS service |
| RPC | `ros2.service.list` | Request-Response | List authorized ROS services |
| RPC | `ros2.topic.list` | Request-Response | List authorized ROS topics |
| RPC | `ros2.interface.show` | Request-Response | Fetch ROS interface definitions |

The bridge has two delivery modes:

- non-video ROS topics are delivered as raw CDR bytes on a LiveKit data track
- ROS image topics(`sensor_msgs/msg/Image` and `sensor_msgs/msg/CompressedImage`) and other video targets are delivered as LiveKit video tracks

The `ros2.*` names mirror the corresponding ROS 2 CLI commands, but request and response bodies
still use the bridge's JSON/CDR protocol rather than CLI text, flags, or YAML.

This means your client needs different expectations for each:

- data-track subscriptions need interface definitions from `ros2.interface.show`
- video subscriptions depend on `video_topic_ids`, `video_other_ids`, and the matching `video.topics.*` / `video.other.*` configuration

The full contract lives in [docs/protocol.md](./docs/protocol.md).

## Before you start

You need:

- a ROS 2 workspace where you can build and source this package
- a reachable LiveKit deployment
- a pre-minted startup token provided via `livekit.token` or `LIVEKIT_TOKEN` env variable.
- at least one allowed topic or service in `access.rules.*.allow`

For standard package build and test commands, see [DEVELOPING.md](./DEVELOPING.md).

## Quickstart

1. Copy the example parameters file.

   ```bash
   cp livekit_bridge.params.example.yaml livekit_bridge.params.yaml
   ```

2. Edit `livekit_bridge.params.yaml` and set the connection, at least one allow rule, and either `livekit.token` or the `LIVEKIT_TOKEN` env variable.

   ```yaml
   livekit_ros2_bridge:
     ros__parameters:
       livekit.url: "wss://your-livekit.example"
       livekit.token: ""  # optional if LIVEKIT_TOKEN is set

       access.rules.subscribe.allow: ["/camera/*"]
       access.rules.publish.allow: ["/cmd_vel"]
       access.rules.service.allow: ["/example/service"]
   ```

   For the full parameter model, including access rules, read [docs/configuration.md](./docs/configuration.md).

3. Run the node.

   ```bash
   export LIVEKIT_TOKEN="your-pre-minted-token"
   ros2 run livekit_ros2_bridge livekit_ros2_bridge_node --ros-args \
     --params-file $(pwd)/livekit_bridge.params.yaml
   ```

   Wait for `event=runtime_ready`.

4. Connect a LiveKit client to the same room and use the resources you allowed.

For a fast first success, allow one service or one small topic path first. Add more rules and video sources after that works.

## Current scope

Supported today:

- ROS service calls
- topic and service discovery
- topic subscriptions
- video subscriptions
- small ROS topic publications into ROS 2
- other video sources

Not supported today:

- ROS actions
- ROS parameter get and set
- full audio support
- large topic publish payloads
- Replaying latched topics
- Metric reporting

## License

Apache License 2.0
