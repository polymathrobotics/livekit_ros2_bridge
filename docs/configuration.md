# Configuration Guide

The bridge reads ROS parameters once at node startup and builds one immutable runtime snapshot. SDK reconnect attempts reuse that snapshot. Changing parameters later does not change live behavior.

If a change affects LiveKit connection settings, access rules, QoS override matching, or video source resolution, restart the node.

## Contents

- [Reference](#reference)
  - [LiveKit auth](#livekit-auth)
  - [Watchdog](#watchdog)
  - [Access rules](#access-rules)
  - [Video](#video)
    - [Defaults](#defaults)
    - [ROS topics](#ros-topics)
    - [Other video sources](#other-video-sources)
  - [QoS](#qos)
- [Common scenarios](#common-scenarios)
  - [RTSP or device inputs](#rtsp-or-device-inputs)

## Reference

### LiveKit auth

| Parameter | Default | Required | Allowed values | Notes |
| --- | --- | --- | --- | --- |
| `livekit.url` | `""` | yes | non-empty string | LiveKit server URL |
| `livekit.token` | `""` | yes* | string | Startup token for the bridge participant. If non-empty, it wins over `LIVEKIT_TOKEN`. |
| `LIVEKIT_TOKEN` | unset | yes* | string | Fallback startup token from the `LIVEKIT_TOKEN` environment variable when `livekit.token` is empty |

\* One of `livekit.token` or `LIVEKIT_TOKEN` must be non-empty.

Notes:

- startup fails if `livekit.url` is empty
- startup fails if both token sources are empty
- `LIVEKIT_TOKEN` is an environment variable, not a ROS parameter

### Watchdog

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `health.watchdog.enabled` | `true` | `true`, `false` | Enables shutdown after the recovery timeout expires |
| `health.watchdog.recovery_timeout_seconds` | `75.0` | double `>= 0.0` | Maximum time the bridge may spend recovering connectivity before it exits |

Notes:

- when the recovery timeout expires, the bridge logs the failure, shuts down, and exits non-zero
- the default 75-second timeout keeps recovery bounded while the SDK owns reconnect

### Access rules

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `access.rules.publish.allow` | `[]` | array of ROS resource patterns | Allowlist for `ros2.topic.pub`. Empty allowlist allows nothing. |
| `access.rules.publish.deny` | `[]` | array of ROS resource patterns | Denylist for `ros2.topic.pub` |
| `access.rules.subscribe.allow` | `[]` | array of ROS resource patterns | Allowlist for topic subscriptions and `ros2.topic.list`. Empty allowlist allows nothing. |
| `access.rules.subscribe.deny` | `[]` | array of ROS resource patterns | Denylist for topic subscriptions and `ros2.topic.list` |
| `access.rules.service.allow` | `[]` | array of ROS resource patterns | Allowlist for `ros2.service.call` and `ros2.service.list`. Empty allowlist allows nothing. |
| `access.rules.service.deny` | `[]` | array of ROS resource patterns | Denylist for `ros2.service.call` and `ros2.service.list` |

Behavior notes:

- deny rules win over allow rules
- rules are global and name-based, not requester-specific
- other video targets do not use `access.rules.subscribe.*`; they are controlled by `video_other_ids` and `video.other.*`
- a forbidden topic subscription is reported as `forbidden` in `lkros.status`

Pattern notes:

- `*` matches the entire operation
- `/foo/bar` matches exactly `/foo/bar`
- `/foo/*` matches descendants under `/foo` but not `/foo` itself
- names are normalized before matching: surrounding whitespace is trimmed, repeated `/` collapses, a missing leading `/` is added, and trailing `/` is removed except for `/`
- empty strings in the arrays are ignored

### Video

#### Defaults

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `video.publish.codec` | `auto` | `auto`, `vp8`, `h264`, `av1`, `vp9`, `h265` | Global LiveKit codec setting |
| `video.publish.max_bitrate_bps` | `0` | integer `>= 0` | Global LiveKit max bitrate in bps |
| `video.publish.max_framerate` | `0.0` | double `>= 0.0` | Global LiveKit max framerate |
| `video.publish.simulcast` | `auto` | `auto`, `enabled`, `disabled` | Global LiveKit simulcast setting |

Publish default notes:

- these defaults expose the same publish controls described in LiveKit's [video track configuration guide](https://docs.livekit.io/transport/media/advanced/) and C++ [`TrackPublishOptions` reference](https://docs.livekit.io/reference/client-sdk-cpp/structlivekit_1_1TrackPublishOptions.html); the bridge forwards them directly when it publishes a video track
- these defaults apply to ROS video topics, other video sources, and the built-in ROS fallback rule
- `auto` or `0` means "use LiveKit SDK default behavior" for that field
- entry-level overrides merge per field with these global defaults

#### ROS topics

These entries apply to ROS topics, not other video sources.

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `video_topic_ids` | `[]` | array of ids | ROS video topic ids defined under `video.topics.<id>.*` |
| `video.topics.<id>.pattern` | none | ROS topic pattern | Required topic pattern for this entry |
| `video.topics.<id>.transform` | `""` | GStreamer middle fragment | Optional processing stages inserted after bridge-managed ROS ingress |
| `video.topics.<id>.publish.codec` | `""` | `""`, `auto`, `vp8`, `h264`, `av1`, `vp9`, `h265` | Empty inherits `video.publish.codec` |
| `video.topics.<id>.publish.max_bitrate_bps` | `-1` | integer `>= -1` | `-1` inherits `video.publish.max_bitrate_bps` |
| `video.topics.<id>.publish.max_framerate` | `-1.0` | double `>= -1.0` | `-1.0` inherits `video.publish.max_framerate` |
| `video.topics.<id>.publish.simulcast` | `""` | `""`, `auto`, `enabled`, `disabled` | Empty inherits `video.publish.simulcast` |

Topic notes:

- duplicate ids in `video_topic_ids` are rejected at startup
- every entry under `video.topics.<id>.*` must have a matching id in `video_topic_ids`
- `video_topic_ids` stays at the root because `generate_parameter_library` 0.6 cannot nest it in the supported distro matrix
- all matching entries are considered
- the longest matching pattern wins
- same-length ties keep declaration order
- if no user entry matches, the built-in `default_ros` entry matches `/*` with no extra `transform`
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

#### Other video sources

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `video_other_ids` | `[]` | array of ids | Other video ids defined under `video.other.<id>.*` |
| `video.other.<id>.source` | none | non-empty GStreamer ingress fragment | Required ingress fragment such as `uridecodebin`, `v4l2src`, or `videotestsrc` |
| `video.other.<id>.transform` | `""` | GStreamer middle fragment | Optional processing stages inserted after the ingress |
| `video.other.<id>.publish.codec` | `""` | `""`, `auto`, `vp8`, `h264`, `av1`, `vp9`, `h265` | Empty inherits `video.publish.codec` |
| `video.other.<id>.publish.max_bitrate_bps` | `-1` | integer `>= -1` | `-1` inherits `video.publish.max_bitrate_bps` |
| `video.other.<id>.publish.max_framerate` | `-1.0` | double `>= -1.0` | `-1.0` inherits `video.publish.max_framerate` |
| `video.other.<id>.publish.simulcast` | `""` | `""`, `auto`, `enabled`, `disabled` | Empty inherits `video.publish.simulcast` |

Other video notes:

- duplicate ids in `video_other_ids` are rejected at startup
- every entry under `video.other.<id>.*` must have a matching id in `video_other_ids`
- `video_other_ids` stays at the root because `generate_parameter_library` 0.6 cannot nest it in the supported distro matrix
- `source` is required and must be non-empty after trimming
- `source` should start with a concrete ingress element such as `uridecodebin`, `v4l2src`, or `videotestsrc`
- `transform` is optional and sits between your ingress and the bridge-owned tail
- the bridge always appends `queue max-size-buffers=2 leaky=downstream ! videoconvert ! video/x-raw,format=I420 ! appsink`
- neither `source` nor `transform` may define `appsrc` or `appsink`; the bridge owns those endpoints
- startup validates `source` plus `transform` with GStreamer, but runtime failures can still happen when a stream actually starts
- explicit entry values of `auto` or `0` still count as overrides and reset that field back to LiveKit SDK default behavior

Lookup notes:

- clients request these sources with `kind: "other_video"` and the source id as `name`
- lookup trims only surrounding whitespace from the requested name
- other video track names percent-encode bytes outside RFC 3986 unreserved characters
- other video sources are not gated by `access.rules.subscribe.*`; availability is controlled by which ids exist in `video_other_ids` and `video.other.*`

### QoS

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `subscription_qos_ids` | `[]` | array of ids | QoS ids to load from `subscription.qos.<id>.*` |
| `subscription.qos.<id>.pattern` | `""` | ROS topic pattern | Required for each referenced id |
| `subscription.qos.<id>.reliability` | `auto` | `auto`, `reliable`, `best_effort` | Reliability override for matching topics |
| `subscription.qos.<id>.durability` | `auto` | `auto`, `volatile`, `transient_local` | Durability override for matching topics |

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

## Common scenarios

### RTSP or device inputs

Use `video.other.*` when the bridge should ingest video directly from GStreamer instead of subscribing to an existing ROS `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage` topic.

1. Define one or more other video ids and give each one a `source` fragment.

   ```yaml
   livekit_ros2_bridge:
     ros__parameters:
       video_other_ids: ["front_rtsp", "usb_cam"]

       video.other.front_rtsp.source: "uridecodebin uri=rtsp://127.0.0.1:8554/front source::latency=0"
       video.other.front_rtsp.transform: "videoscale ! video/x-raw,width=1280,height=720"
       video.other.front_rtsp.publish.codec: "h264"

       video.other.usb_cam.source: "v4l2src device=/dev/video0 do-timestamp=true"
       video.other.usb_cam.transform: ""
       video.other.usb_cam.publish.max_framerate: 30.0
   ```

2. Keep the pipeline boundaries in the right place.

   - `source` should start with the ingress stage such as `uridecodebin` or `v4l2src`
   - `transform` is optional and should contain only middle-of-pipeline processing stages
   - do not put `appsrc` or `appsink` into either fragment; the bridge owns those endpoints and appends its own queue/convert/I420/appsink tail
   - leave `video.publish.*` and per-source `publish.*` unset unless you need to force codec, bitrate, framerate, or simulcast behavior
   - other video sources do not need `access.rules.subscribe.allow`; they become available because they are declared in `video_other_ids`

3. Request the source by id from the client with `kind: "other_video"`.

   ```json
   {
     "v": 2,
     "type": "lkros.heartbeat",
     "subscriptions": [
       {
         "kind": "other_video",
         "name": "front_rtsp"
       },
       {
         "kind": "other_video",
         "name": "usb_cam"
       }
     ]
   }
   ```

4. Expect video delivery in `lkros.status`.

   - the `name` is the trimmed source id
   - an active entry reports `delivery.kind: "video"`
   - the track name is deterministic, for example `lkros.video.other.front_rtsp`
   - if the source id contains reserved bytes, the track-name suffix is percent-encoded
   - if a client asks for a source that does not exist, the bridge reports `not_found`
