# Video sources and sidecars

The bridge treats every video subscription as a request to resolve one canonical `SidecarLaunchSpec` and keep one managed publisher process alive for that spec's `sidecar_key`.

That path starts from one of two inputs:

- a ROS topic whose resolved interface type is a supported video type
- an `external` target backed by a configured `videos.*` pipeline entry

## What becomes a video stream

Configured `external` targets are always video.

ROS topics become video only when their resolved interface type is one of:

- `sensor_msgs/msg/Image`
- `sensor_msgs/msg/CompressedImage`

Those two ROS types choose the pipeline alias and ingest mode:

| Interface type | Pipeline alias | Ingest mode |
| --- | --- | --- |
| `sensor_msgs/msg/Image` | `image` | `raw_image` |
| `sensor_msgs/msg/CompressedImage` | `compressed_image` | `compressed_image` |

All other ROS topic types stay on the CDR data-track path.

## ROS topic rules

`videos.<id>.kind=ros` entries become topic-matching rules. Each rule has:

- an `id`
- a normalized topic pattern
- one or more pipeline templates keyed by `image`, `compressed_image`, or `default`

Rule selection works like this:

- normalize the requested topic name
- find every matching rule
- choose the longest matching pattern
- keep the first declared rule when two matching patterns have the same length

Pipeline selection works like this:

1. use the interface-specific alias, `image` or `compressed_image`
2. fall back to `default`

If neither pipeline exists, resolution fails.

In ROS pipeline templates, every `{topic}` placeholder is replaced with the normalized topic name before the final command is tokenized.

### Built-in ROS fallback

If no user rule matches, the built-in `default_ros` rule handles both supported ROS video types. Its defaults start from:

- `rosrawimagesrc` for `sensor_msgs/msg/Image`
- `roscompressedimagesrc ! jpegdec` for `sensor_msgs/msg/CompressedImage`

Both defaults add:

- a small leaky queue
- `videorate` capped at `12/1`
- `videoconvert`
- `vp8enc`

## Configured external sources

`videos.<id>.kind=pipeline` entries become configured external sources.

Rules:

- only the `default` pipeline alias is accepted
- duplicate ids that normalize to the same external name are rejected at startup
- lookup uses the normalized external name

That means values such as `front_rtsp`, `/front_rtsp`, and `/front_rtsp/` all collapse to the same canonical external name, `"/front_rtsp"`.

Configured external pipelines are tokenized as-is. They do not interpolate `{topic}`.

## Sidecar prerequisites

Bridge-managed video sidecars depend on a few runtime requirements:

| Requirement | Why it matters |
| --- | --- |
| `livekit.api_key` and `livekit.api_secret` | Sidecars mint their own publisher tokens |
| `gstreamer-publisher` on `PATH` | The bridge launches it as the managed publisher process |
| `GST_PLUGIN_PATH` including this package's installed plugin directory for source builds | Lets `gstreamer-publisher` find `rosrawimagesrc` and `roscompressedimagesrc` |

The packaged runtime image and the repo dev container set up the GStreamer plugin path for you. Source builds need to export it before starting the bridge so child sidecar processes inherit it.

## Sidecar lifecycle

`VideoSidecarSupervisor` owns one child process per resolved `sidecar_key`. `SubscriptionRegistry` calls `ensureSidecar()` when a video subscription is created or renewed.

Three rules matter most:

- the map is keyed by `sidecar_key`, so normalized requests share one child
- publisher identity is derived once per key and reused across respawns
- calling `ensureSidecar()` again for an already-running sidecar reuses the child instead of starting a second one

Publisher identities are deterministic:

- ROS topics use `<bridge_identity>-video-<topic slug>`
- configured external sources use `<bridge_identity>-video-source-<external slug>`

The command shape is:

```text
gstreamer-publisher --url <livekit_url> --token <publisher_token> -- <source_pipeline...>
```

The supervisor does not run its own background poll loop. Maintenance happens when the runtime calls `maintainSidecars()` during lease sweeping.

Each maintenance cycle:

1. reaps exited children
2. restarts unhealthy publishers when a health check callback is configured
3. restarts children whose minted publisher token is approaching expiry

Restart behavior is intentionally conservative:

- replacement argv and token are prepared before the current child is killed
- if command building or token minting fails, the current child is left running
- health checks are ignored during startup grace
- a sidecar restarts only after enough consecutive failed health checks
- token refresh lead time is clamped to at most half of the token TTL

Bridge-managed sidecars exist only when the bridge can mint publisher tokens. A static `livekit.token` without `livekit.api_key` and `livekit.api_secret` is enough for the bridge itself to join LiveKit, but not enough to launch sidecars.

Subscription status for video uses `publisher_identity`. `track_name` is empty for video today.

## Shutdown and process groups

Each sidecar child calls `setpgid(0, 0)` after `fork()`, so it runs in its own process group.

On stop or shutdown, the supervisor:

- sends `SIGTERM` to the process group
- waits briefly for exit
- escalates to `SIGKILL` if the process group is still alive
- falls back to signaling the direct child only if process-group signaling fails because the group no longer exists

That keeps wrapper scripts and grandchildren from being left behind.

## Built-in ROS GStreamer sources

The built-in `rosrawimagesrc` and `roscompressedimagesrc` elements are the default source stage for ROS-backed video.

Shared behavior:

- each element creates its own ROS node and subscription on `NULL -> READY`
- each uses `SensorDataQoS()` unless `ros-reliable=true`
- each keeps at most one queued message, so delivery is latest-frame-wins
- `getcaps()` waits for the first message, derives caps from that sample, and then drops that sampled frame
- once caps are fixed, neither element renegotiates mid-stream
- `unlock()` wakes blocked `getcaps()` and `create()` calls; `unlock_stop()` clears the flushing state again

`rosrawimagesrc` supports only these ROS image encodings:

- `mono8`
- `mono16`
- `rgb8`
- `bgr8`
- `rgba8`
- `bgra8`
- `yuv422`
- `yuv422_yuy2`

If a later `sensor_msgs/msg/Image` message changes width, height, or encoding after caps negotiation, `create()` returns `GST_FLOW_ERROR` instead of renegotiating.

`roscompressedimagesrc` supports only JPEG and PNG payloads. It parses the primary token from `CompressedImage.format`, ignoring any `; ...` suffix, trimming whitespace, and lowercasing before matching. Once the first frame picks JPEG or PNG, a later format change also becomes `GST_FLOW_ERROR`.
