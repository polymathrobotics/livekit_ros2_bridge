# Configuration Guide

The bridge reads ROS parameters once at node startup and builds one immutable runtime snapshot. Reconnects reuse that snapshot. Changing parameters later does not change live behavior.

If a change affects LiveKit connection settings, access rules, QoS override matching, or video source resolution, restart the node. Reconnect alone is not enough.

## Table of contents

- [Reference](#reference)
- [Common scenarios](#common-scenarios) - how-to section at the bottom
- [Set up RTSP or physical-device video inputs](#set-up-rtsp-or-physical-device-video-inputs)

## Reference

### LiveKit connection and authentication

| Setting | Default | Required | Allowed values | Notes |
| --- | --- | --- | --- | --- |
| `livekit.url` | `""` | yes | non-empty string | LiveKit server URL |
| `livekit.token` | `""` | yes* | string | Startup token for the bridge participant. If non-empty, it wins over `LIVEKIT_TOKEN`. |
| `LIVEKIT_TOKEN` | unset | yes* | string | Fallback startup token when `livekit.token` is empty |

\* One of `livekit.token` or `LIVEKIT_TOKEN` must be non-empty.

Notes:

- startup fails if `livekit.url` is empty
- startup fails if both token sources are empty
- the resolved startup token is reused when the bridge creates a fresh room connection
- if that token expires later, reconnect attempts fail until something outside the bridge restarts it with a fresh token

### Health watchdog

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `health.watchdog.enabled` | `true` | `true`, `false` | Enables shutdown after the recovery timeout expires |
| `health.watchdog.recovery_timeout_seconds` | `600.0` | double `>= 0.0` | Maximum time the bridge may spend recovering connectivity before it exits |

Notes:

- the default 10-minute timeout is intentional because long outages can outlive LiveKit's reconnect-token window
- the default 10-minute timeout also comfortably exceeds the current upstream LiveKit C++ SDK reconnect budget: the SDK tries up to 10 reconnect attempts, each of which can spend about 5 seconds opening signal, 5 seconds waiting for the reconnect response, and 15 seconds waiting for ICE, so it can give up after roughly 250 seconds before the bridge watchdog takes over
- that LiveKit reconnect budget is an SDK default, not a bridge parameter exposed in this configuration file
- the recovery timeout starts at startup before the first successful LiveKit connection
- the recovery timeout is cleared after a successful connection
- each reconnect episode arms a fresh recovery timeout
- when the recovery timeout expires, the bridge logs the failure, shuts down, and exits non-zero
- clean shutdown does not trigger the watchdog
- missing required startup settings still fail immediately during config load without waiting for the watchdog

### Access rules

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `access.rules.publish.allow` | `[]` | array of ROS resource patterns | Allowlist for `ros.topics.publish`. Empty allowlist allows nothing. |
| `access.rules.publish.deny` | `[]` | array of ROS resource patterns | Denylist for `ros.topics.publish` |
| `access.rules.subscribe.allow` | `[]` | array of ROS resource patterns | Allowlist for topic subscriptions and `ros.topics.list`. Empty allowlist allows nothing. |
| `access.rules.subscribe.deny` | `[]` | array of ROS resource patterns | Denylist for topic subscriptions and `ros.topics.list` |
| `access.rules.service.allow` | `[]` | array of ROS resource patterns | Allowlist for `ros.services.call` and `ros.services.list`. Empty allowlist allows nothing. |
| `access.rules.service.deny` | `[]` | array of ROS resource patterns | Denylist for `ros.services.call` and `ros.services.list` |

Pattern notes:

- `*` matches the entire operation
- `/foo/bar` matches exactly `/foo/bar`
- `/foo/*` matches descendants under `/foo`
- names are normalized before matching: surrounding whitespace is trimmed, repeated `/` collapses, a missing leading `/` is added, and trailing `/` is removed except for `/`
- empty strings in the arrays are ignored

Behavior notes:

- deny rules win over allow rules
- rules are global and name-based, not requester-specific
- `configured_source` targets do not use `access.rules.subscribe.*`; they are controlled by `video_configured_source_ids` and `video.configured_sources.*`
- a forbidden topic subscription is reported as `forbidden` in `ros.subscriptions.status`

### Subscriber QoS overrides

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `subscription_qos_overrides_ids` | `[]` | array of ids | QoS override ids to load from `subscription.qos_overrides.<id>.*` |
| `subscription.qos_overrides.<id>.pattern` | `""` | ROS topic pattern | Required in practice for each referenced id |
| `subscription.qos_overrides.<id>.reliability` | `auto` | `auto`, `reliable`, `best_effort` | Reliability override for matching topics |
| `subscription.qos_overrides.<id>.durability` | `auto` | `auto`, `volatile`, `transient_local` | Durability override for matching topics |

Resolution notes:

- overrides apply only to ROS topic subscriptions
- they affect both data-track subscriptions and ROS video topic subscriptions
- if multiple overrides match, the longest matching pattern wins
- same-length ties keep declaration order
- each QoS axis resolves independently: explicit override, then visible publisher QoS, then the subscription-class base QoS
- without a matching override, the bridge only infers `reliability` and `durability` from visible publishers
- if publishers disagree, the bridge chooses the weaker compatible policy for that axis
- publisher `unknown` and `system_default` policies do not contribute to inference
- data-track subscriptions start from `KeepLast(2)`
- ROS video topic subscriptions start from `KeepLast(1)`
- QoS is resolved only when the shared subscription is created or recreated later; it is not live-reconciled after that

### Video entry lists

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `video_topic_rule_ids` | `[]` | array of ids | ROS video rule ids defined under `video.topic_rules.<id>.*` |
| `video_configured_source_ids` | `[]` | array of ids | Configured source ids defined under `video.configured_sources.<id>.*` |

Notes:

- these id lists stay at the root because `generate_parameter_library` 0.6 in the current distro matrix cannot move them cleanly under `video.topic_rules.ids` and `video.configured_sources.ids` yet
- duplicate ids in either list are rejected at startup
- each listed id must have matching generated parameters under its `video.topic_rules.<id>.*` or `video.configured_sources.<id>.*` entry
- configured source ids are trimmed for lookup, so surrounding whitespace is ignored but names such as `/front_rtsp` and `/front_rtsp/` remain distinct

### Global video publish defaults

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `video.publish.codec` | `auto` | `auto`, `vp8`, `h264`, `av1`, `vp9`, `h265` | Global LiveKit codec setting |
| `video.publish.max_bitrate_bps` | `0` | integer `>= 0` | Global LiveKit max bitrate in bps |
| `video.publish.max_framerate` | `0.0` | double `>= 0.0` | Global LiveKit max framerate |
| `video.publish.simulcast` | `auto` | `auto`, `enabled`, `disabled` | Global LiveKit simulcast setting |

Notes:

- these defaults apply to ROS video topic rules, configured sources, and the built-in ROS fallback rule
- `auto` or `0` means "use LiveKit SDK default behavior" for that field
- entry-level overrides merge per field with these global defaults

### ROS video topic rules

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `video.topic_rules.<id>.pattern` | none | ROS topic pattern | Required topic pattern for this rule |
| `video.topic_rules.<id>.transform` | `""` | GStreamer middle fragment | Optional processing stages inserted after bridge-managed ROS ingress |
| `video.topic_rules.<id>.publish.codec` | `""` | `""`, `auto`, `vp8`, `h264`, `av1`, `vp9`, `h265` | Empty inherits `video.publish.codec` |
| `video.topic_rules.<id>.publish.max_bitrate_bps` | `-1` | integer `>= -1` | `-1` inherits `video.publish.max_bitrate_bps` |
| `video.topic_rules.<id>.publish.max_framerate` | `-1.0` | double `>= -1.0` | `-1.0` inherits `video.publish.max_framerate` |
| `video.topic_rules.<id>.publish.simulcast` | `""` | `""`, `auto`, `enabled`, `disabled` | Empty inherits `video.publish.simulcast` |

Rule notes:

- all matching rules are considered
- the longest matching pattern wins
- same-length ties keep declaration order
- if no user rule matches, the built-in `default_ros` rule matches `/*` with no extra `transform`
- explicit entry values of `auto` or `0` still count as overrides and reset that field back to LiveKit SDK default behavior

Pipeline notes:

- ROS raw image topic: `appsrc(caps from Image) ! <transform> ! <bridge tail>`
- ROS compressed image topic: `appsrc(image/jpeg|image/png) ! jpegdec|pngdec ! <transform> ! <bridge tail>`
- the bridge tail is always `queue max-size-buffers=2 leaky=downstream ! videoconvert ! video/x-raw,format=I420 ! appsink`
- `transform` should contain only optional middle-of-pipeline stages
- `transform` must not define `appsrc` or `appsink`; the bridge owns those endpoints
- `transform` is validated with GStreamer at startup
- the bridge does not interpolate placeholders such as `{topic}` inside `transform`

Supported ROS video inputs:

- ROS topics must resolve to `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage`
- for `sensor_msgs/msg/Image`, supported encodings are `mono8`, `mono16`, `rgb8`, `bgr8`, `rgba8`, `bgra8`, `yuv422`, and `yuv422_yuy2`
- for `sensor_msgs/msg/CompressedImage`, supported payloads are JPEG and PNG, including image_transport-style format strings that name `jpeg`, `jpg`, or `png`

### Configured video sources

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `video.configured_sources.<id>.source` | none | non-empty GStreamer ingress fragment | Required ingress fragment such as `uridecodebin`, `v4l2src`, or `videotestsrc` |
| `video.configured_sources.<id>.transform` | `""` | GStreamer middle fragment | Optional processing stages inserted after the configured ingress |
| `video.configured_sources.<id>.publish.codec` | `""` | `""`, `auto`, `vp8`, `h264`, `av1`, `vp9`, `h265` | Empty inherits `video.publish.codec` |
| `video.configured_sources.<id>.publish.max_bitrate_bps` | `-1` | integer `>= -1` | `-1` inherits `video.publish.max_bitrate_bps` |
| `video.configured_sources.<id>.publish.max_framerate` | `-1.0` | double `>= -1.0` | `-1.0` inherits `video.publish.max_framerate` |
| `video.configured_sources.<id>.publish.simulcast` | `""` | `""`, `auto`, `enabled`, `disabled` | Empty inherits `video.publish.simulcast` |

Source notes:

- `source` is required and must be non-empty after trimming
- `source` should start with the ingress stage, such as RTSP, V4L2, or a test source
- `transform` is optional and sits between your ingress and the bridge-owned tail
- the bridge always appends `queue max-size-buffers=2 leaky=downstream ! videoconvert ! video/x-raw,format=I420 ! appsink`
- neither `source` nor `transform` may define `appsrc` or `appsink`; the bridge owns those endpoints
- startup validates `source` plus `transform` with GStreamer, but runtime failures can still happen when a stream actually starts
- explicit entry values of `auto` or `0` still count as overrides and reset that field back to LiveKit SDK default behavior

Lookup notes:

- clients request configured sources by `configured_source` name
- lookup trims only surrounding whitespace from the requested name
- configured source track names percent-encode bytes outside RFC 3986 unreserved characters
- configured sources are not gated by `access.rules.subscribe.*`; availability is controlled by which ids exist in `video_configured_source_ids` and `video.configured_sources.*`

### Debug video profiling

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `debug.video_profiling.enabled` | `false` | `true`, `false` | Enables bridge-side video profiling at startup |
| `debug.video_profiling.summary_interval_ms` | `5000` | integer `> 0` | Interval for aggregated profiling summary logs |
| `debug.video_profiling.trace_file` | `"/workspace/log/video-profile.trace.json"` | non-empty string | Trace output path written on shutdown when profiling is enabled |
| `debug.video_profiling.trace_max_events` | `250000` | integer `> 0` | Maximum retained in-memory trace events before older events are dropped |

Notes:

- profiling is configured at startup only
- when enabled, the bridge logs the profiling config, emits periodic summaries, and flushes the trace file on shutdown

## Common scenarios

### Set up RTSP or physical-device video inputs

Use `video.configured_sources.*` when the bridge should ingest video directly from GStreamer instead of subscribing to an existing ROS `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage` topic.

1. Define one or more configured source ids and give each one a `source` fragment.

   ```yaml
   livekit_ros2_bridge:
     ros__parameters:
       video_configured_source_ids: ["front_rtsp", "usb_cam"]

       video.configured_sources.front_rtsp.source: "uridecodebin uri=rtsp://127.0.0.1:8554/front source::latency=0"
       video.configured_sources.front_rtsp.transform: "videoscale ! video/x-raw,width=1280,height=720"
       video.configured_sources.front_rtsp.publish.codec: "h264"

       video.configured_sources.usb_cam.source: "v4l2src device=/dev/video0 do-timestamp=true"
       video.configured_sources.usb_cam.transform: ""
       video.configured_sources.usb_cam.publish.max_framerate: 30.0
   ```

2. Keep the pipeline boundaries in the right place.

   - `source` should start with the ingress stage such as `uridecodebin` or `v4l2src`
   - `transform` is optional and should contain only middle-of-pipeline processing stages
   - do not put `appsrc` or `appsink` into either fragment; the bridge owns those endpoints and appends its own queue/convert/I420/appsink tail
   - leave `video.publish.*` and per-source `publish.*` unset unless you need to force codec, bitrate, framerate, or simulcast behavior
   - configured sources do not need `access.rules.subscribe.allow`; they become available because they are declared in `video_configured_source_ids`

3. Request the configured source by id from the client.

   ```json
   {
     "v": 2,
     "type": "ros.subscriptions.heartbeat",
     "subscriptions": [
       {
         "kind": "configured_source",
         "name": "front_rtsp"
       },
       {
         "kind": "configured_source",
         "name": "usb_cam"
       }
     ]
   }
   ```

4. Expect video delivery in `ros.subscriptions.status`.

   - the `name` is the trimmed configured source id
   - an active entry reports `delivery.kind: "video"`
   - the track name is deterministic, for example `ros.video.configured_source.front_rtsp`
   - if the configured source name contains reserved bytes, the track-name suffix is percent-encoded
   - if a client asks for a configured source that does not exist, the bridge reports `not_found`
