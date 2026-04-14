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

## Access policy parameters

Access rules are also startup-only:

- `access.rules.publish.allow`
- `access.rules.publish.deny`
- `access.rules.subscribe.allow`
- `access.rules.subscribe.deny`
- `access.rules.service.allow`
- `access.rules.service.deny`

Empty allow lists are valid, but they deny everything for that operation. Deny rules always win over allow rules.

For matching behavior, read [access-control.md](./access-control.md).

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

Example:

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

## Video entries

Video entries are loaded only at startup.

Common parameters:

| Parameter | Meaning |
| --- | --- |
| `video_topic_rule_ids` | the topic rule ids to load from `video.topic_rules.<id>` |
| `video_configured_source_ids` | the configured source ids to load from `video.configured_sources.<id>` |
| `video.topic_rules.<id>.pattern` | topic pattern for this ROS video rule |
| `video.topic_rules.<id>.transform` | optional transform fragment for this ROS video rule |
| `video.topic_rules.<id>.publish.*` | optional partial LiveKit publish overrides for this ROS video rule |
| `video.configured_sources.<id>.source` | ingress fragment for this configured source |
| `video.configured_sources.<id>.transform` | optional transform fragment for this configured source |
| `video.configured_sources.<id>.publish.*` | optional partial LiveKit publish overrides for this configured source |

Rules:

- `video.topic_rules.<id>.pattern` is required
- `video.configured_sources.<id>.source` is required
- `transform` is optional for both entry types
- `publish.*` is optional for both entry types and can override any subset of `video.publish.*`
- duplicate `video_topic_rule_ids` entries are rejected
- duplicate `video_configured_source_ids` entries are rejected
- configured source ids trim to the `configured_source` names clients request; slash and colon variants stay distinct

Examples:

- `video.topic_rules.front_camera.pattern: "/camera/front/*"` matches ROS topics according to that pattern
- `video.topic_rules.front_camera.publish.max_framerate: 15.0` overrides only framerate for that rule
- `video.configured_sources.front_rtsp.source: "uridecodebin uri=rtsp://..."` creates a configured source requested as `{"kind":"configured_source","name":"front_rtsp"}`
- `video.configured_sources.front_rtsp.publish.codec: "h264"` overrides only codec for that source

`video_topic_rule_ids` and `video_configured_source_ids` stay at the root for now because the generate_parameter_library 0.6 baseline in the current distro matrix cannot move them cleanly under `video.topic_rules.ids` and `video.configured_sources.ids` yet.

Default LiveKit video publish options are also startup-only:

- `video.publish.codec`
- `video.publish.max_bitrate_bps`
- `video.publish.max_framerate`
- `video.publish.simulcast`

Those settings provide the default for every video track publish. Leave them at their defaults to use SDK-selected behavior, or override any subset per entry with `video.topic_rules.<id>.publish.*` or `video.configured_sources.<id>.publish.*`.

## Runtime image

The repo runtime image uses the same parameter file shape as a workspace run.

By default it reads `/config/livekit_bridge.params.yaml`. Set `LIVEKIT_BRIDGE_PARAMS_FILE` to use a different in-container path.

The runtime image also exports `GST_PLUGIN_PATH` for the installed `gstrosbridge` plugin, so built-in ROS video source readers do not need extra plugin-path setup there.

For rule resolution and video runtime behavior, read [video-sources.md](./video-sources.md). For the LiveKit-facing contract, read [protocol.md](./protocol.md) and [subscriptions.md](./subscriptions.md).
