# Runtime configuration

The bridge reads ROS parameters once at node startup and builds one immutable runtime snapshot. Reconnects reuse that snapshot. Changing parameters later does not change live behavior.

At startup, configuration determines:

- how the bridge connects to LiveKit
- which ROS resources each access policy family exposes
- how video requests resolve to ROS-backed or configured sources

If a change would affect any of those decisions, restart the node. Reconnect alone is not enough.

## Required connection settings

| Parameter | Required | Notes |
| --- | --- | --- |
| `livekit.url` | yes | LiveKit server URL |
| `livekit.room` | yes | Room name the bridge should join |

Startup fails if `livekit.url` or `livekit.room` is empty.

## Authentication

| Parameter | Required | Notes |
| --- | --- | --- |
| `livekit.token` | yes | Startup token for the bridge participant. The bridge does not mint or refresh tokens. |

Startup fails if `livekit.token` is empty.

The bridge reuses the configured startup token when it creates a fresh room connection. If that token expires, later reconnect attempts will fail until something outside the bridge restarts it with a fresh token.

## Watchdog health parameters

The bridge can shut itself down when recovery takes too long. This is intended for environments where something outside the bridge may decide what to do next, such as restarting it with a fresh startup token.

The 10-minute default is intentional: long outages can outlive LiveKit's refreshed reconnect-token window.

| Parameter | Default | Notes |
| --- | --- | --- |
| `health.watchdog.enabled` | `true` | Enables shutdown after the recovery timeout expires |
| `health.watchdog.recovery_timeout_seconds` | `600.0` | Maximum time the bridge may spend recovering connectivity before it exits |

Important behavior:

- the recovery timeout starts at startup before the first successful LiveKit connection
- the recovery timeout is cleared after a successful connection
- each reconnect episode arms a fresh recovery timeout
- when the recovery timeout expires, the bridge logs the failure, shuts down, and exits non-zero
- clean shutdown does not trigger the watchdog

This is separate from static config validation. Missing `livekit.url`, `livekit.room`, or `livekit.token` still fail immediately at startup without waiting for the recovery timeout.

## Access rules

Access rules are startup-only.

Parameters:

- `access.rules.publish.allow`
- `access.rules.publish.deny`
- `access.rules.subscribe.allow`
- `access.rules.subscribe.deny`
- `access.rules.service.allow`
- `access.rules.service.deny`

Which rules apply:

| Operation | Parameters | Used by |
| --- | --- | --- |
| Publish | `access.rules.publish.allow`, `access.rules.publish.deny` | `ros.topics.publish` |
| Subscribe | `access.rules.subscribe.allow`, `access.rules.subscribe.deny` | topic entries in `ros.subscriptions.heartbeat`, `ros.topics.list` |
| Service | `access.rules.service.allow`, `access.rules.service.deny` | `ros.services.call`, `ros.services.list` |

Pattern syntax:

- `*` matches everything for that operation
- `/foo/bar` matches exactly `/foo/bar`
- `/foo/*` matches descendants under `/foo`

Rules:

- deny rules win over allow rules
- an empty allow list allows nothing
- rules are name-based, not requester-specific
- `configured_source` targets are controlled by `video_configured_source_ids` and `video.configured_sources.*`, not by `access.rules.subscribe.*`

Surface notes:

- `ros.topics.list` uses the same subscribe rules as topic subscriptions
- `ros.services.list` uses the same service rules as `ros.services.call`
- a forbidden topic subscription is reported as `forbidden` in `ros.subscriptions.status`

## Subscriber QoS resolution

ROS topic subscriptions resolve subscriber QoS when the bridge creates each shared subscription.

Default behavior:

- inspect visible publishers for that topic first
- infer only `reliability` and `durability`
- keep data-track subscriptions at `KeepLast(2)`
- keep ROS video subscriptions at `KeepLast(1)`
- if any publisher is `best_effort`, subscribe `best_effort`
- otherwise subscribe `reliable`
- if any publisher is `volatile`, subscribe `volatile`
- otherwise subscribe `transient_local`
- if no usable publisher QoS is visible yet, fall back to the subscription class default

QoS is resolved only when the subscription is created or recreated later. It is not live-reconciled after that.

### QoS override parameters

Use `subscription_qos_overrides_ids` together with `subscription.qos_overrides.*` to pin subscriber QoS fields for ROS topic subscriptions.

| Parameter | Meaning |
| --- | --- |
| `subscription_qos_overrides_ids` | the override ids to load |
| `subscription.qos_overrides.<id>.pattern` | ROS topic pattern to match |
| `subscription.qos_overrides.<id>.reliability` | `auto`, `reliable`, or `best_effort` |
| `subscription.qos_overrides.<id>.durability` | `auto`, `volatile`, or `transient_local` |

Override rules:

- overrides apply only to ROS topic subscriptions
- they affect both data-track and ROS video subscriptions
- longest matching pattern wins
- same-length ties keep declaration order
- `auto` defers to publisher inspection for that field

Examples:

```yaml
livekit_ros2_bridge:
  ros__parameters:
    subscription_qos_overrides_ids: ["gazebo_cameras"]
    subscription.qos_overrides.gazebo_cameras.pattern: "/front_camera/*"
    subscription.qos_overrides.gazebo_cameras.reliability: "reliable"
    subscription.qos_overrides.gazebo_cameras.durability: "auto"
```

This is useful when a publisher is not visible yet at subscription-creation time, or when a source such as `ros_gz_bridge` cameras must be pinned to `reliable`.

## Publish cache limit

The bridge keeps up to `50` active ROS publishers cached for `ros.topics.publish`.

Important behavior:

- the bridge creates generic publishers on demand
- when the cache grows past the limit, the least recently used cached publisher is evicted after the current publish succeeds

## Video

Video configuration is startup-only.

### Parameter layout

Use root-level `video_topic_rule_ids` and `video_configured_source_ids`, with matching entries under `video.topic_rules.<id>.*` and `video.configured_sources.<id>.*`.

These ids stay at the root because `generate_parameter_library` 0.6 in the current distro matrix cannot move them cleanly under `video.topic_rules.ids` and `video.configured_sources.ids` yet.

Duplicate ids in either list are rejected at startup.

### ROS topic rules

A `video.topic_rules.<id>` entry defines one ROS video rule.

| Parameter | Meaning |
| --- | --- |
| `video.topic_rules.<id>.pattern` | required ROS topic pattern |
| `video.topic_rules.<id>.transform` | optional middle-of-pipeline GStreamer fragment |
| `video.topic_rules.<id>.publish.*` | optional per-rule overrides for `video.publish.*` |

Rule selection:

- collect every matching rule
- choose the longest matching pattern
- keep declaration order when two matches have the same pattern length

If no user rule matches, the built-in `default_ros` rule handles supported ROS video topics with no extra `transform` stages.

The bridge does not interpolate placeholders such as `{topic}` inside `transform`.

### Configured sources

A `video.configured_sources.<id>` entry defines one named source. Clients request it as a `configured_source` by id.

| Parameter | Meaning |
| --- | --- |
| `video.configured_sources.<id>.source` | required ingress fragment |
| `video.configured_sources.<id>.transform` | optional middle-of-pipeline GStreamer fragment |
| `video.configured_sources.<id>.publish.*` | optional per-source overrides for `video.publish.*` |

Rules:

- `source` is required and must be non-empty
- `transform` is optional
- `publish.*` is optional and can override any subset of `video.publish.*`
- names such as `/front_rtsp` and `front_rtsp/` stay distinct

### Publish defaults and overrides

Default LiveKit publish options:

- `video.publish.codec`
- `video.publish.max_bitrate_bps`
- `video.publish.max_framerate`
- `video.publish.simulcast`

Each topic rule or configured source can override any subset of those fields with `video.topic_rules.<id>.publish.*` or `video.configured_sources.<id>.publish.*`.

Override behavior:

- overrides merge per field with `video.publish.*`
- omitted entry fields inherit the default
- explicit entry values of `auto` or `0` still count as overrides and reset that field back to LiveKit SDK default behavior
- when neither the global setting nor the entry forces an override, the bridge leaves that setting to the LiveKit SDK

With the built-in ROS default and no publish overrides, the bridge publishes incoming resolution and frame cadence as-is.

### How `source` and `transform` are used

The bridge always appends its own output tail:

```text
queue max-size-buffers=2 leaky=downstream ! videoconvert ! video/x-raw,format=I420 ! appsink
```

Composition by source kind:

- ROS raw image topic: `appsrc(caps from Image) ! <transform> ! <bridge tail>`
- ROS compressed image topic: `appsrc(image/jpeg|image/png) ! jpegdec|pngdec ! <transform> ! <bridge tail>`
- configured source: `<source> ! <transform> ! <bridge tail>`

Rules:

- ROS `transform` should contain only optional processing stages in the middle of the pipeline
- configured-source `source` should start with the ingress stage, such as RTSP, V4L2, or a test source
- `transform` should not create `appsrc` or `appsink`; the bridge owns those endpoints

Startup validates `source` and `transform` with GStreamer. Runtime failures still happen only when a stream starts.

### Supported ROS video inputs

For `sensor_msgs/msg/Image`, the bridge accepts only ROS encodings that `rosImageEncodingToGstFormat()` maps to a known GStreamer raw format:

- `mono8`
- `mono16`
- `rgb8`
- `bgr8`
- `rgba8`
- `bgra8`
- `yuv422`
- `yuv422_yuy2`

For `sensor_msgs/msg/CompressedImage`, the bridge accepts only JPEG and PNG payloads. It understands either plain codec names such as `jpeg` and `png` or image_transport-style format strings such as `rgb8; jpeg compressed bgr8`.
