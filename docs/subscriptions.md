# Subscriptions

`ros.subscriptions.heartbeat` is a lease renewal packet, not a one-time start command. Each heartbeat says "this is the full set of subscriptions I still want right now" for one requester. The bridge renews 45-second leases, updates shared runtime state, and publishes one `ros.subscriptions.status` packet when the heartbeat contains at least one entry.

## Heartbeat request

Control topic: `ros.subscriptions.heartbeat`

```json
{
  "session_id": "tab-123",
  "subscriptions": [
    {
      "kind": "topic",
      "name": "/battery_state",
      "delivery_preferences": {
        "interval_ms": 100
      }
    },
    {
      "kind": "configured_source",
      "name": "front_camera"
    }
  ]
}
```

Rules:

- `subscriptions` is required and must be an array
- each entry must be an object
- each entry must contain string `kind` and `name` fields
- `kind` must be `topic` or `configured_source`
- `topic` names are normalized as ROS resource names
- `configured_source` names are trimmed only, so surrounding whitespace is ignored but `/`, repeated `/`, trailing `/`, and `:` stay significant
- if topic normalization or configured-source trimming produces an empty canonical name, that entry is invalid
- `delivery_preferences` is optional and must be an object when present
- `delivery_preferences.interval_ms` is optional and must be an integer when present
- `session_id` is optional; missing, `null`, and blank values are treated as absent

## Duplicate targets and interval handling

The heartbeat parser treats `subscriptions` as a set keyed by canonical `kind` + `name`:

- repeating the same canonical target yields one effective request
- when duplicates provide multiple non-zero `interval_ms` values, the smallest value wins
- `interval_ms: 0` means no preference
- negative `interval_ms` values are accepted by the parser but clamped to `0` when the lease is applied

For data-track subscriptions carrying ROS CDR payloads, the bridge later computes one applied interval per shared stream by taking the minimum requested interval across all current requesters.

## Requester identity and `session_id`

The bridge prefers the LiveKit packet's `requester_identity`. `session_id` exists only to keep heartbeats working when LiveKit omits that identity from user-data packets.

Behavior:

- heartbeats with a non-empty `requester_identity` are accepted normally
- if that heartbeat also includes `session_id`, the bridge binds the `session_id` to that requester for 45 seconds
- a later heartbeat with an empty `requester_identity` is accepted only if it includes a known, unexpired `session_id`
- a `session_id` cannot be rebound to a different requester until the existing lease expires
- session leases are separate from stream leases

Anonymous heartbeats without a known `session_id` are dropped. A heartbeat with an empty `subscriptions` array renews nothing and produces no status packet.

## Status response

Control topic: `ros.subscriptions.status`

```json
{
  "v": 2,
  "type": "ros.subscriptions.status",
  "session_id": "tab-123",
  "lease_expires_in_ms": 44980,
  "subscriptions": [
    {
      "kind": "topic",
      "name": "/battery_state",
      "status": "active",
      "interface_type": "sensor_msgs/msg/BatteryState",
      "delivery": {
        "kind": "data",
        "track_name": "ros.data.battery_state",
        "content_type": "application/x-ros-cdr",
        "interval_ms": 100
      }
    }
  ]
}
```

Envelope rules:

- `v` is the protocol version and is currently `2`
- `type` is always `ros.subscriptions.status`
- `session_id` and `lease_expires_in_ms` are included only when the heartbeat carried a non-blank `session_id`
- `lease_expires_in_ms` is computed at serialization time, so it is approximate
- the bridge publishes no status packet when the heartbeat produces an empty `subscriptions` array

## Subscription entries

Active entries always include:

- `kind`: `topic` or `configured_source`
- `name`
- `status`: `active`

Error entries always include:

- `kind`
- `name`
- `status`: `error`
- `error.reason`
- `error.message`

Current `error.reason` values are:

- `forbidden`: the subscribe policy denies a topic
- `unavailable`: the bridge could not start or keep running a required runtime dependency, usually a video stream pipeline
- `not_found`: lookup or subscription creation failed for another reason

## Active topic subscription entries on a data track

Non-video ROS topics are delivered on a LiveKit data track:

```json
{
  "kind": "topic",
  "name": "/battery_state",
  "status": "active",
  "interface_type": "sensor_msgs/msg/BatteryState",
  "delivery": {
    "kind": "data",
    "track_name": "ros.data.battery_state",
    "content_type": "application/x-ros-cdr",
    "interval_ms": 100
  }
}
```

Notes:

- `track_name` is deterministic: the bridge prefixes `ros.data` and replaces `/` with `.`
- `delivery.interval_ms` is always present for data deliveries, including `0`
- the bytes sent on the data track are raw serialized CDR bytes, not nested JSON

## Active video subscription entries

Video deliveries use the same deterministic track-name surface the bridge publishes:

```json
{
  "kind": "configured_source",
  "name": "front_camera",
  "status": "active",
  "delivery": {
    "kind": "video",
    "track_name": "ros.video.configured_source.front_camera"
  }
}
```

Notes:

- `configured_source` targets are always video streams
- ROS topics become video streams only when their resolved type is `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage`
- video `track_name` is deterministic and stable for the canonical target name
- configured-source video track names percent-encode any byte outside RFC 3986 unreserved characters, so ids that differ by `/` or `:` stay distinct

## Delivery and sharing model

| Requested target | Resolved from | Delivery | Shared resource model |
| --- | --- | --- | --- |
| Non-video ROS topic | normalized topic name and unique graph type | data | one ROS subscription and one data track per normalized topic |
| ROS video topic | normalized topic name, unique graph type, and matching video rule | video | one in-process video stream per resolved `stream_key` |
| Configured source | trimmed `configured_source` name and matching `video.configured_sources.*` entry | video | one in-process video stream per resolved `stream_key` |

Important behavior:

- topic subscriptions use `access.rules.subscribe.*`
- `configured_source` targets do not use subscribe rules; they are controlled by which `video_configured_source_ids` and `video.configured_sources.*` entries exist
- the bridge never guesses when topic type resolution is ambiguous
- when the last requester lease disappears, the shared data track or video stream is torn down

## Lease timing and cleanup

- each successful heartbeat renews each requested subscription for 45 seconds from processing time
- the runtime sweeps expired session leases and stream leases once per second
- `lease_expires_in_ms` reflects the remaining time at the moment the status packet is serialized
- data-track delivery uses one applied interval per shared topic, based on the smallest current requester interval

## Reconnect and data-track republish

Browser refresh and transport reconnect are not treated the same way.

When a requester disconnects outside reconnect handling, the bridge can keep that requester's leases alive. If that requester still owns published data tracks, the next successful heartbeat may force those tracks through an unpublish and republish cycle under the same deterministic `track_name`.

That replay exists because LiveKit data-track publications belong to a participant session. The lease may still be valid even though the old publication belonged to the disconnected session.

Two practical consequences:

- the first few ROS samples can be dropped while a data track is still pending publication
- later samples can still be dropped if the LiveKit data-track queue is full
