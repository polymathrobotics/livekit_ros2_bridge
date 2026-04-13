# Video sources

The bridge resolves each video subscription to one canonical in-process `VideoStreamSpec` and keeps one shared `VideoStreamRegistry` runtime alive for that spec's `stream_key`.

Video requests start from one of two inputs:

- a ROS topic whose resolved interface type is a supported video type
- a `configured_source` target backed by a configured `video.configured_sources.*` entry

## What becomes a video stream

`configured_source` targets are always video.

ROS topics become video only when their resolved interface type is one of:

- `sensor_msgs/msg/Image`
- `sensor_msgs/msg/CompressedImage`

Those two ROS types choose only the ingest mode:

| Interface type | Ingest mode |
| --- | --- |
| `sensor_msgs/msg/Image` | `raw_image` |
| `sensor_msgs/msg/CompressedImage` | `compressed_image` |

All other ROS topic types stay on the data-track path carrying ROS CDR payloads.

## Public config shape

Use root-level `video_topic_rule_ids`, `video_configured_source_ids`, and the matching `video.topic_rules.<id>.*` / `video.configured_sources.<id>.*` entries.

`video_topic_rule_ids` and `video_configured_source_ids` stay at the root for now because the generate_parameter_library 0.6 baseline in the current distro matrix cannot move them cleanly to `video.topic_rules.ids` and `video.configured_sources.ids` yet.

Each topic rule has:

- `pattern`: required
- `transform`: optional
- `publish.*`: optional partial overrides for `video.publish.*`

Each configured source has:

- `source`: required
- `transform`: optional
- `publish.*`: optional partial overrides for `video.publish.*`

## ROS topic rules

`video.topic_rules.<id>` entries named in `video_topic_rule_ids` become topic-matching rules. Each rule has:

- an `id`
- a normalized topic pattern
- one optional `transform` fragment
- zero or more optional `publish.*` overrides

Rule selection works like this:

- normalize the requested topic name
- find every matching rule
- choose the longest matching pattern
- keep the first declared rule when two matching patterns have the same length

ROS `transform` is a pure middle-stage fragment. The bridge does not interpolate `{topic}` or any other placeholders.

### Built-in ROS fallback

If no user rule matches, the built-in `default_ros` rule handles both supported ROS video types.

Its `transform` is empty, so the default behavior is bridge-managed ingress plus bridge-managed tail, with no extra processing stages.

## Configured sources

`video.configured_sources.<id>` entries named in `video_configured_source_ids` become configured sources.

Rules:

- `source` is required and must be non-empty
- `transform` is optional
- `publish.*` fields are optional and may override any subset of `video.publish.*`
- duplicate ids that trim to the same configured-source name are rejected at startup
- lookup uses the trimmed configured-source name

That means values such as `front_rtsp` and ` front_rtsp ` collapse to the same canonical configured-source name, `front_rtsp`, while `/front_rtsp` and `front_rtsp/` stay distinct.

Configured-source video track names use the trimmed id directly after percent-encoding any byte outside RFC 3986 unreserved characters. For example, `/front_rtsp:rgb` becomes `ros.video.configured_source.%2Ffront_rtsp%3Argb`.

## Runtime pipeline composition

The bridge always appends its own output tail:

```text
queue max-size-buffers=2 leaky=downstream ! videoconvert ! video/x-raw,format=I420 ! appsink
```

Composition by source kind:

- ROS raw image topic: `appsrc(caps from Image) ! <transform> ! <bridge tail>`
- ROS compressed image topic: `appsrc(image/jpeg|image/png) ! jpegdec|pngdec ! <transform> ! <bridge tail>`
- configured source: `<source> ! <transform> ! <bridge tail>`

That means:

- ROS `transform` should describe only optional processing stages
- configured-source `source` should begin with the ingress stage, such as RTSP, V4L2, or a test source
- `transform` should not create `appsrc` or `appsink`; the bridge owns those endpoints

At startup the bridge uses GStreamer itself to parse and structurally validate configured `source` and `transform` fragments. That catches malformed syntax and forbidden endpoint ownership early, but runtime failures such as bad URIs, negotiation problems, EOS, or source outages can still happen only when the stream starts.

## LiveKit publish defaults and overrides

These startup-only parameters define the default LiveKit track publish options for video streams:

- `video.publish.codec`
- `video.publish.max_bitrate_bps`
- `video.publish.max_framerate`
- `video.publish.simulcast`

Each `video.topic_rules.<id>` and `video.configured_sources.<id>` entry may optionally override any subset of those fields with:

- `video.topic_rules.<id>.publish.codec`
- `video.topic_rules.<id>.publish.max_bitrate_bps`
- `video.topic_rules.<id>.publish.max_framerate`
- `video.topic_rules.<id>.publish.simulcast`
- `video.configured_sources.<id>.publish.codec`
- `video.configured_sources.<id>.publish.max_bitrate_bps`
- `video.configured_sources.<id>.publish.max_framerate`
- `video.configured_sources.<id>.publish.simulcast`

Entry overrides merge per field with `video.publish.*`, so omitted entry fields inherit the default. Explicit entry values of `auto` or `0` still count as overrides and reset that field back to LiveKit SDK default behavior.

When both the default and the entry override stay at their defaults, the bridge does not force an override and the LiveKit SDK uses its own defaults.

In the default ROS case with no override and no publish overrides:

- the bridge publishes the incoming frame resolution as-is
- frame cadence follows the source
- codec, bitrate, framerate, and simulcast follow LiveKit SDK defaults

## Shared stream lifecycle

`VideoStreamRegistry` owns one in-process runtime per resolved `stream_key`.

Important behavior:

- the map is keyed by normalized `stream_key`, so equivalent requests share one stream
- calling `start()` again for an existing key reuses the current runtime
- when the last requester lease disappears, the shared runtime is torn down

Failure handling differs slightly by source kind:

- ROS streams recreate their internal pipeline when image dimensions or compressed format change
- configured-source streams restart after pipeline EOS or error, with a short backoff

## ROS input constraints

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
