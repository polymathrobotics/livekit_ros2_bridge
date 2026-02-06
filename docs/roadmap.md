# Roadmap

This document captures planned future directions for `livekit_ros2_bridge`.
It is intentionally forward-looking; the current client contract is in `docs/protocol.md`.

## Problem statement

Robots often operate in network-constrained environments. The bridge should:

- Minimize uplink bandwidth from the robot.
- Support many browser clients (primary) plus occasional server/tooling clients.
- Use LiveKit primitives that are built for high-rate media and large binary payloads.

## Current state (today)

- Protocol: JSON over LiveKit data packets
  - LiveKit → ROS: `ros.topic.publish`
  - ROS → LiveKit: `ros.topic.messages`
  - Control: RPC `ros.topic.subscribe`, `ros.topic.unsubscribe`, `ros.service.call`
- Data packet size constraints (practical limits)
  - Reliable packets: ~15KiB max user payload (protocol limit 16KiB).
  - Lossy packets: recommended ~1300 bytes max to stay under MTU.
- Subscriptions are currently targeted to active requesters (not broadcast to the room).
- Image/point-cloud types are blocked by default for ROS → LiveKit subscriptions.

## Direction: split control plane vs media/data plane

### Control plane: RPC + small JSON packets

Keep the existing JSON protocol for:

- Commands and small telemetry.
- Subscribe/unsubscribe/service calls.

Add ROS discovery APIs for UIs (candidate RPCs; names TBD):

- `ros.list_topics`, `ros.list_services` (access-filtered).
- (Later) node/param discovery and parameter get/set.

### Media/data plane: LiveKit video tracks + byte streams

Add separate mechanisms for high-rate or large payloads:

- Video tracks for live cameras (bandwidth-first; uses LiveKit video pipeline).
- Byte streams for:
  - JPEG image frames when lower encode latency is preferred.
  - Generic binary payloads beyond the data packet size limits.

## Proposed milestones

### Phase 1 — ROS discovery + QoS knobs

- Add access-filtered RPCs: topic/service listing.
- Extend `ros.topic.subscribe` to accept QoS hints (e.g., reliability and depth) for sensor topics.

### Phase 2 — JPEG image streaming (non-video)

- Add image subscription RPCs (candidate: `ros.topic.subscribe_image` / `ros.topic.unsubscribe_image`) with `mode="jpeg"`.
- Transport: LiveKit byte streams with `mimeType="image/jpeg"`.
- Include stream attributes like `{ros_topic, ros_type, stamp, frame_id}`.
- Prefer selective delivery to active requesters when supported.

### Phase 3 — Video-track streaming (bandwidth-first)

- Reuse image subscription RPCs with `mode="video"`.
- Publish one LiveKit video track per ROS camera topic.
- Expose safe uplink tuning knobs (max bitrate, max fps).
- Enforce per-requester permissions on who may subscribe to the track.

### Phase 4 — Generic binary streaming (“bytes mode”)

- Add binary subscription RPCs (candidate: `ros.topic.subscribe_bytes` / `ros.topic.unsubscribe_bytes`).
- Serialize ROS messages to bytes (e.g., ROS 2 CDR) and send via LiveKit byte streams.
- Add stream attributes `{ros_topic, ros_type, encoding, stamp, seq}`.

### Phase 5 — Follow-ons (future)

- Client → robot binary uploads (strict access rules + sandboxing).
- Action support (ROS actions) and parameter set/get.
- Recording / capture workflows (e.g., snapshot to file, short clips).

## Policy and limits (required for new features)

- Add allow/deny lists for image and bytes streaming.
- Add limits: max active streams (total/per participant), max fps/bitrate, max in-flight writers.
- Keep default-deny; denylist overrides allowlist.

## Known constraints to design around

- Data packets are best for small messages; LiveKit docs recommend streams/RPC for most uses.
- Byte streams are real-time only (no server buffering/history); participants joining mid-stream do not receive prior data, and streams can optionally target specific participants (destination identities). Chunks are generally smaller than 15kB. ([docs.livekit.io](https://docs.livekit.io/transport/data/byte-streams/))
- Video publishing requires continuously sending frames (even for static images) so late joiners can render; LiveKit’s track publishing APIs use `VideoSource`/`VideoFrame`. ([docs.livekit.io](https://docs.livekit.io/home/client/tracks/publish))

## References

- LiveKit data packets: https://docs.livekit.io/transport/data/packets/
- LiveKit byte streams: https://docs.livekit.io/transport/data/byte-streams/
- LiveKit track publishing (video): https://docs.livekit.io/home/client/tracks/publish
