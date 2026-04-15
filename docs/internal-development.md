# Internal Development Guide

This guide is for internal code, docs, tests, comments, and logs. It does not define the wire
protocol. For external payloads and field names, see [protocol.md](./protocol.md).

## Running tests

From the repository root:

```bash
just test
```

The `just` wrapper builds first, then runs the suite inside the repository's dev-container workflow.

To run the same `colcon` unit-test suite across every supported ROS distro with
ephemeral Docker builds:

```bash
just test-matrix
```

## Logging

The bridge uses flat structured logs built with `LogEvent`: each line starts with `event=...` and
then adds `key=value` pairs.

- Keep `event` names descriptive.
- Keep fields minimal. Include the fields that answer what happened, to what, and why.

## Terminology

Use these words and phrases consistently in `livekit_ros2_bridge`.

- `cdr`: Payload encoding only. Never the transport.
- `config`: Startup configuration loaded from parameters. It does not change while the bridge is running.
- `connection`: The bridge's connection to a LiveKit room, including reconnect handling.
- `control message`: A bridge protocol message sent on a LiveKit data-packet topic. Use it for heartbeats, status, and topic publish requests, not for streamed ROS topic data.
- `data packet`: LiveKit `publishData` payload. The bridge uses data packets for control messages and topic publish requests.
- `demand`: One requested subscription carried inside a heartbeat.
- `drain`: Run or empty queued work on the owning thread.
- `heartbeat`: Control message that renews one requester's subscription leases.
- `inflight`: Active unresolved requests that count against a quota.
- `lease`: A time-limited claim that keeps a subscription alive.
- `participant`: LiveKit room member.
- `payload`: Message envelope and serialization helpers.
- `quiesce`: Stop admitting new callbacks while letting already-started work finish.
- `registry`: Shared manager that owns deduplicated resources.
- `resource`: Real ROS graph entity used for lookup or access checks.
- `room`: LiveKit room and membership container. Use `room connection` for the bridge wrapper.
- `session`: A requester or participant over time. Do not use it for the bridge's room connection.
- `source`: The input-side object that produces frames or messages for a stream such as a video frame source.
- `spec`: The resolved inputs and settings built from a `config` plus a specific request or lookup.
- `track`: LiveKit transport object. Prefer an explicit track type when possible.
  - `data track`: LiveKit track for continuous binary data.
  - `video track`: LiveKit video track.

### Data (ROS -> LiveKit)

- `instance`: One per-topic ROS-to-LiveKit bridge. It owns the ROS subscription and
  LiveKit data-track publication.
- `subscription`: ROS `rclcpp::GenericSubscription` input side.
- `publisher`: The object that publishes, unpublishes, and writes to a LiveKit data track, such
  as `DataTrackPublisher`.
- `stream`: The end-to-end path from a ROS subscription to a LiveKit data track.

### Video (ROS / GStreamer -> LiveKit)

- `instance`: One shared bridge for one resolved video stream. It owns the frame source and
  LiveKit video-track publication, such as `VideoStreamInstance`.
- `source`: The object that produces frames for a video stream, such as `VideoFrameSource`.
- `publisher`: The object that publishes one LiveKit video track and accepts frames for it, such
  as `VideoTrackPublisher`.
- `track`: LiveKit transport object only, such as `livekit::LocalVideoTrack` or `PublishedVideoTrack`.
- `stream`: The end-to-end path from a ROS or configured video source to a LiveKit video track.

### Boundaries & Practical Rules

- `callbacks` vs `handlers`: Use `callbacks` for caller-supplied functions invoked later, and `handlers` for code that owns the handling behavior.
- `on` vs `handle`: Prefer `on*` for methods. Prefer `*Handler` for types and standalone callable values. Avoid bare nouns or bare verbs for callback slots.
- `config` vs `spec`: Use `config` for declared parameters and `spec` for resolved inputs and settings.
- `cdr` vs `track` vs `stream`: `cdr` is encoding, `track` is LiveKit transport, and `stream` is the bridge's end-to-end delivery path.
- `track types`: Prefer `data track`, `video track`, and `audio track`. Use bare `track` only when the type is obvious from the surrounding code.
- `scope provides context`: In short functions or well-named types/namespaces, prefer short role-based locals and bare verbs when the surrounding scope already makes the domain obvious. Prefer `body` to `responseBody` unless extra words are needed to disambiguate.
- `preserve the role noun`: When shortening a type or field name, remove redundant qualifiers before removing the semantic role word. Prefer `Rules` over `OperationRules`, but keep `RuleEntries` instead of shortening it to `Entries`.
- `resource` vs `target`: `resource` is a real ROS graph entity. `target` may also be a configured source.
- `topic`: In LiveKit packet code, `topic` means the LiveKit data-packet topic. Use `ros_topic` or `lk_topic` when both concepts are nearby.
- `connection` vs `session` vs `lease`: `connection` is the bridge's room connection, `session`
  is a requester or participant over time, and `lease` is a time-limited claim.
