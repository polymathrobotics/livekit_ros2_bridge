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
  - [Audio](#audio)
    - [Defaults](#audio-defaults)
    - [Other audio sources](#other-audio-sources)
    - [Audio output](#audio-output)
  - [QoS](#qos)
- [Common scenarios](#common-scenarios)
  - [RTSP or device inputs](#rtsp-or-device-inputs)
  - [Cab microphones](#cab-microphones)
  - [Audio output playback](#audio-output-playback)

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

### Audio

#### Audio defaults

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `audio.publish.max_bitrate_bps` | `0` | int `>= 0` | Global LiveKit max audio bitrate override in bps. `0` uses SDK defaults. |
| `audio.publish.dtx` | `auto` | `auto`, `enabled`, `disabled` | Global LiveKit DTX override. |
| `audio.publish.red` | `auto` | `auto`, `enabled`, `disabled` | Global LiveKit RED override. |

Notes:

- `auto` leaves the LiveKit SDK default (off) in place
- per-source `publish.*` values override these globals; `-1` (bitrate) or `""` (dtx/red) inherit the global

#### Other audio sources

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `audio_other_ids` | `[]` | array of ids | Other audio ids to load from `audio.other.<id>.*` |
| `audio.other.<id>.source` | — | GStreamer fragment | Required. Ingress fragment for this other audio source (e.g. `audiotestsrc`, `pulsesrc`). |
| `audio.other.<id>.transform` | `""` | GStreamer fragment | Optional transform fragment inserted after other audio ingress |
| `audio.other.<id>.publish.max_bitrate_bps` | `-1` | int `>= -1` | Optional LiveKit max audio bitrate override in bps. `-1` inherits from `audio.publish.max_bitrate_bps`. |
| `audio.other.<id>.publish.dtx` | `""` | `""`, `auto`, `enabled`, `disabled` | Optional LiveKit DTX override. Empty inherits from `audio.publish.dtx`. |
| `audio.other.<id>.publish.red` | `""` | `""`, `auto`, `enabled`, `disabled` | Optional LiveKit RED override. Empty inherits from `audio.publish.red`. |

Notes:

- duplicate ids in `audio_other_ids` are rejected at startup
- every entry under `audio.other.<id>.*` must have a matching id in `audio_other_ids`
- `audio_other_ids` stays at the root because `generate_parameter_library` 0.6 cannot nest it in the supported distro matrix
- `source` is required and must be non-empty after trimming
- `source` should start with a concrete ingress element such as `audiotestsrc` or `pulsesrc`
- `transform` is optional and sits between your ingress and the bridge-owned tail
- the bridge always appends `queue max-size-time=100ms leaky=downstream ! audioconvert ! audioresample ! audio/x-raw,format=S16LE,channels=1,rate=48000 ! appsink`
- neither `source` nor `transform` may define `appsrc` or `appsink`; the bridge owns those endpoints
- startup validates `source` plus `transform` with GStreamer, but runtime failures can still happen when a stream actually starts
- every configured source publishes as exactly one mono audio track; stereo is client-side routing of two mono tracks

Lookup notes:

- clients request these sources with `kind: "other_audio"` and the source id as `name`
- lookup trims only surrounding whitespace from the requested name
- other audio track names percent-encode bytes outside RFC 3986 unreserved characters
- other audio sources are not gated by `access.rules.subscribe.*`; availability is controlled by which ids exist in `audio_other_ids` and `audio.other.*`

#### Audio output

| Parameter | Default | Allowed values | Notes |
| --- | --- | --- | --- |
| `audio.out.sink` | `""` | non-empty GStreamer sink fragment | Audio output playback pipeline. Empty (default) disables audio output entirely. |

Behavior notes:

- audio output is disabled by default; the feature never half-works on robots without an output device
- a non-empty `audio.out.sink` enables the feature and advertises `audio.out`, with the `track_name` to publish, through the [`lkros.capability`](protocol.md#rpc-lkroscapability) RPC, so client UIs offer audio output only where it can work
- the bridge connects with auto-subscribe disabled and subscribes only to the fixed-name audio output track (`lkros.audio.out`); other published media is never received
- the bridge performs no identity checks on the audio output publisher; who may publish is enforced by the application layer
- the playback pipeline is `appsrc ! audioconvert ! audioresample ! <audio.out.sink>`; appsrc is the only bridge-owned buffer and holds at most 200 ms, dropping the oldest audio beyond that, so an output that falls behind skips instead of building up delay; the fragment is inserted verbatim after those bridge-owned stages
- `audio.out.sink` must not define `appsrc` or `appsink`; the bridge owns those endpoints
- playback is paced by arrival: the bridge sets `sync=false` on every sink in the fragment, overriding the fragment's own setting, so outputs that buffer more than they declare (e.g. `alsasink` through the ALSA pulse plugin) still play
- output-device failures restart the pipeline at a bounded rate (~4/s) and only while audio is arriving; a missing device never crashes the node, never cycles while idle, and self-heals when audio arrives with the device restored
- mute is silence-through: no bridge reaction, the sink stays claimed until the track is unpublished

Fragment notes:

- raw ALSA (recommended v1 deployment): `alsasink device=...` with the device passed through into the container
- a sound server (e.g. `pulsesink server=...`): needs the socket/environment plumbing in the deployment inventory
- a networked audio device: any GStreamer sink fragment that terminates in an audio device
- clock-disciplined sinks (anything syncing to the pipeline clock, e.g. `pulsesink` without `sync=false`) crackle at the 10 ms cadence; end the fragment with `sync=false` (e.g. `pulsesink sync=false`) unless the sink is clock-disciplined by design

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

### Cab microphones

Use `audio.other.*` when the bridge should ingest audio directly from GStreamer instead of subscribing to a ROS audio topic. Each configured source becomes one mono audio track; route two tracks to left/right speakers for a stereo-operator experience.

1. Define one or more other audio ids and give each one a `source` fragment.

   ```yaml
   livekit_ros2_bridge:
     ros__parameters:
       audio_other_ids: ["left_mic", "right_mic"]

       audio.other.left_mic.source: "pulsesrc device=alsa_input.pci-0000_00_1f.3.analog-stereo"
       audio.other.left_mic.transform: "volume volume=0.8"
       audio.other.left_mic.publish.max_bitrate_bps: 64000

       audio.other.right_mic.source: "pulsesrc device=alsa_input.pci-0000_00_1f.3.analog-stereo"
       audio.other.right_mic.publish.dtx: "enabled"
   ```

2. Keep the pipeline boundaries in the right place.

   - `source` should start with the ingress stage such as `audiotestsrc` or `pulsesrc`
   - `transform` is optional and should contain only middle-of-pipeline processing stages
   - do not put `appsrc` or `appsink` into either fragment; the bridge owns those endpoints and appends its own queue/convert/resample/mono-48k/appsink tail
   - leave `audio.publish.*` and per-source `publish.*` unset unless you need to force bitrate, DTX, or RED behavior
   - other audio sources do not need `access.rules.subscribe.allow`; they become available because they are declared in `audio_other_ids`

3. Request the source by id from the client with `kind: "other_audio"`.

   ```json
   {
     "v": 2,
     "type": "lkros.heartbeat",
     "subscriptions": [
       {
         "kind": "other_audio",
         "name": "left_mic"
       },
       {
         "kind": "other_audio",
         "name": "right_mic"
       }
     ]
   }
   ```

4. Expect audio delivery in `lkros.status`.

   - the `name` is the trimmed source id
   - an active entry reports `delivery.kind: "audio"`
   - the track name is deterministic, for example `lkros.audio.other.left_mic`
   - if the source id contains reserved bytes, the track-name suffix is percent-encoded
   - if a client asks for a source that does not exist, the bridge reports `not_found`

### Audio output playback

Set `audio.out.sink` when the bridge should play client-published audio through an output device on the robot. The feature is off unless the fragment is configured.

1. Configure the output device as one free-form GStreamer sink fragment.

   ```yaml
   livekit_ros2_bridge:
     ros__parameters:
       audio.out.sink: "alsasink device=hw:0,0"
   ```

   - the fragment is deployment's choice: raw ALSA, a sound server, or a networked audio device — bridge code is identical either way
   - the sink's GStreamer plugin is deployment's to install: `alsasink` needs the ALSA plugin (`gstreamer1.0-alsa` on Debian/Ubuntu) in the runtime image; without it, startup fails with `no element "alsasink"`
   - do not put `appsrc` or `appsink` into the fragment; the bridge owns those endpoints and prepends its own appsrc/convert/resample stages
   - the container needs access to the audio device (e.g. `devices: ["/dev/snd"]` in the service spec); that is deployment inventory, not a bridge parameter

2. Expect the feature to be discoverable.

   - `lkros.capability` answers `{"v": 2, "features": {"audio": {"out": {"track_name": "lkros.audio.out"}}}}`
   - with `audio.out.sink` empty (or absent), `features` is `{}` and no track is ever subscribed

3. Expect the client side to be self-enforcing.

   - the client publishes its audio track as `lkros.audio.out` while it holds the control lease and unpublishes on lease loss; the bridge rebinds its sink to the next output track's first frame
   - a second live output track is logged and dropped; it never steals the sink
   - the bridge performs no identity checks on the publisher

4. Expect bounded, self-healing playback.

   - a missing output device restarts the pipeline at ~4/s only while audio arrives, never on an idle robot, and never crashes the node
   - plugging the output device in mid-stream restores audio output without a node restart
