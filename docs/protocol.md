# Client Interface and Protocol Specification

This document specifies the client-facing interface and protocol of `livekit_ros2_bridge`.

## Introduction

`livekit_ros2_bridge` is a ROS 2 node that joins a LiveKit room and makes the local ROS 2 graph available to other participants in that room. Speaking both ROS 2 and LiveKit, the bridge translates for the client. This document defines the contract between the bridge and those clients.

The bridge supports four kinds of interaction:

- **Live subscriptions** to ROS topics, delivered on LiveKit [data tracks](https://docs.livekit.io/intro/basics/rooms-participants-tracks/) or, for image topics, [video tracks](https://docs.livekit.io/intro/basics/rooms-participants-tracks/).
- **Fire-and-forget publishes** to allowed ROS topics, over LiveKit [data packets](https://docs.livekit.io/home/client/data/packets/).
- **Request/response** against ROS services, over LiveKit [RPC](https://docs.livekit.io/home/client/data/rpc/).
- **Non-ROS video sources** configured on the bridge, delivered on video tracks.
- **Audio output**: the client publishes a fixed-name audio track, the bridge plays it out of a configured sink.

This specification covers the client-facing surface of that contract: the RPC methods the bridge exposes, the data-packet topics it accepts, the data-packet topics and media it emits, the remote media track it consumes, and the shared request, response, authorization, and delivery semantics behind them.

A conforming client MUST follow the schemas and behavioral rules below, and MUST NOT rely on any behavior this document does not specify.

### Conventions

`MUST`, `MUST NOT`, `SHOULD`, and `MAY` carry their usual normative meaning.

Every section is normative unless its heading begins with **Informative:**, or it is a subsection titled **Example** or **Notes**. Informative material illustrates or motivates the normative rules but does not add requirements.

## Scope

### In Scope

- The wire format of LiveKit data-packet topic messages the bridge accepts and produces.
- The wire format of LiveKit RPC requests and responses the bridge accepts and produces.
- The naming, content type, and payload format of data tracks and video tracks the bridge publishes.
- The naming and media format of the remote audio track the bridge consumes for audio output.
- Identity, authorization, and session handling as observed by clients.
- Subscription lease lifetime and reconnect behavior.
- The error vocabulary surfaced to clients.

### Not Covered

This specification does not define:

- how LiveKit access tokens are issued, signed, or validated;
- how the bridge joins a LiveKit room or how rooms are provisioned;
- the bridge's configuration file schema, including how `access.rules.*`, `video.other.*`, and similar policies are authored;
- the ROS 2 discovery, DDS, or networking configuration the bridge runs under;
- operational concerns such as logging format, metrics, or deployment topology.

Clients MUST treat these as implementation details and MUST NOT rely on any of them.

## Versioning and Terminology

### Version

The [`lkros.status`](#data-packet-topic-lkrosstatus) packet and the [`lkros.capability`](#rpc-lkroscapability) response carry a protocol version field `v`, currently `2`.

### Terms

- **bridge**: the `livekit_ros2_bridge` participant in the LiveKit room.
- **canonical name**: the `(kind, name)` pair used to identify a heartbeat target after `topic` names are normalized and `other_video` / `other_audio` names are trimmed
- **client**: the non-bridge participant interacting with the bridge over LiveKit.
- **control-plane message**: a bridge-directed message about subscription lease or status state.
- **data-packet topic**: the LiveKit topic string on a `publishData` packet (outside RPC).
- **data track**: a LiveKit data publication carrying raw ROS CDR bytes.
- **lease**: the bridge's time-bounded record that a client still wants a subscription.
- **normalized name**: a ROS name converted to absolute form before validation and policy checks.
- **ROS publish request**: a bridge-accepted request to publish one message to a ROS topic.
- **ROS resource name**: a normalized ROS topic or service name accepted as a valid resource identifier by the bridge.
- **video track**: a LiveKit video publication carrying a ROS-backed or GStreamer-backed stream.
- **audio track**: a LiveKit audio publication carrying one configured other-audio source as a mono stream.
- **audio output track**: a LiveKit audio publication a client publishes with the fixed name `lkros.audio.out`; the only remote media track the bridge subscribes to.

## Protocol Surfaces

Every surface in this specification runs over LiveKit. Requests and control flows use RPCs or data packets; streams use data, video, or audio tracks.

| Type | Name | Flow | Purpose |
| --- | --- | --- | --- |
| Data-Packet Topic | `lkros.heartbeat` | client → bridge | Request and renew subscriptions |
| Data-Packet Topic | `lkros.status` | bridge → client | Report per-subscription status |
| Data-Packet Topic | `ros2.topic.pub` | client → bridge | Best-effort ROS topic publication |
| Data Track | `delivery.track_name` | bridge → client | Stream active non-video ROS topics |
| Video Track | `delivery.track_name` | bridge → client | Stream ROS image topics or `other_video` sources |
| Audio Track | `delivery.track_name` | bridge → client | Stream `other_audio` sources |
| Byte Stream | `lkros.echo.once` | bridge → client | Deliver a topic's cached last message on request |
| RPC | `ros2.interface.show` | client ↔ bridge | Fetch interface definitions |
| RPC | `ros2.service.call` | client ↔ bridge | Call an authorized ROS service |
| RPC | `ros2.service.list` | client ↔ bridge | List authorized ROS services |
| RPC | `ros2.topic.list` | client ↔ bridge | List authorized ROS topics |
| RPC | `ros2.topic.echo.once` | client ↔ bridge | Request a topic's cached last message |
| RPC | `lkros.capability` | client ↔ bridge | Discover optional bridge features |
| Audio Track | `lkros.audio.out` | client → bridge | Audio output playback (feature-gated) |

Data-track and video-track names are not fixed strings. Clients learn them from an active [`lkros.status`](#data-packet-topic-lkrosstatus) entry and subscribe to the LiveKit publication with that name.

## ROS Payload Envelope

Any ROS binary payload inside a JSON request or response MUST use this envelope:

```json
{
  "content_type": "application/x-ros-cdr",
  "payload_base64": "AAECAw=="
}
```

### Requirements

- `content_type` MUST be `application/x-ros-cdr`.
- `payload_base64` MUST be padded standard base64.
- Wrong `content_type`, wrong field types, or invalid base64 MUST be rejected.
- Parsers MUST trim string fields before validation.
- Blank optional strings SHOULD be treated as absent unless an operation says otherwise.
- Unknown JSON fields SHOULD be ignored unless an operation says otherwise.
- Whether a decoded payload may be empty is operation-specific; operations that require a non-empty payload say so.
- This envelope does not apply to data-track frames, which carry raw serialized CDR bytes.

### Where Used

The same envelope appears in:

- `ros2.topic.pub.message`
- `ros2.service.call.request`
- `ros2.service.call.response`

## Error Model

Errors reach clients through two distinct channels, and the choice of channel is part of the contract.

### Failure Domains

| Domain | How the client learns of failure |
| --- | --- |
| Malformed data-packet topic messages, unsupported data-packet topics, and anonymous data-packet writes | Logged by the bridge and dropped. No reply. |
| Malformed subscription heartbeats | Logged by the bridge and dropped. No `lkros.status` reply. |
| Well-formed heartbeats containing individually failing subscription targets | Per-target entry on `lkros.status` with `status: "error"`. |
| Well-formed heartbeats containing unrecognized subscription kinds | Unrecognized entries produce a per-target `unsupported_kind` error entry on `lkros.status`; remaining targets are processed. |
| RPC failures | LiveKit RPC error, with a code from the table below. |

The bridge MUST NOT invent a reply channel for a domain that does not have one. In particular, a malformed [`ros2.topic.pub`](#data-packet-topic-ros2topicpub) packet MUST NOT produce an `lkros.status` entry or any other acknowledgement.

### Stable RPC Error Codes

| Code | Meaning | Typical causes |
| --- | --- | --- |
| `2400` | invalid request | bad JSON, wrong field types, empty canonical names, payload decode failures, out-of-range values |
| `2401` | unauthorized | missing `caller_identity` |
| `2403` | forbidden | access policy denies the requested service or topic |
| `2500` | internal | ROS graph failures, client creation failures, runtime exceptions, late service-call failures |

Input validation errors (bad arguments, out-of-range values) MUST map to `2400`. All other unhandled errors MUST map to `2500` unless the handler has already raised a more specific code.

These codes are stable. New codes MAY be added in later protocol versions; clients SHOULD treat unrecognized codes as equivalent to `2500`.

### `lkros.status` Error Reasons

When a heartbeat target fails individually, the corresponding [`lkros.status`](#data-packet-topic-lkrosstatus) entry carries `status: "error"` and an `error` object with `reason` and `message` fields. The `message` is informative; `reason` is drawn from this enum:

| Reason | Meaning |
| --- | --- |
| `forbidden` | subscribe policy denies the topic |
| `not_found` | lookup or subscription creation failed for another reason |
| `unsupported_kind` | the bridge does not support the entry's subscription kind |

New reasons MAY be added in later versions of this specification; clients SHOULD treat unrecognized reasons as equivalent to `not_found`. This document describes the unreleased protocol version 2, and `unsupported_kind` is part of it: version 2 has never shipped, so adding the reason does not break a released contract.

### Example

If a client sends malformed `ros2.topic.pub` JSON, the bridge logs and drops it with no reply. If the client sends a valid heartbeat for a forbidden topic, the bridge still replies on `lkros.status`, but that entry has `status: "error"` and reason `forbidden`. If the client calls `ros2.service.call` without `caller_identity`, the call fails with RPC code `2401`.

## Delivery and Sharing Model

| Requested target | Resolved from | Delivery | Shared resource model |
| --- | --- | --- | --- |
| Non-video ROS topic | [normalized](#versioning-and-terminology) topic name and unique graph type | data track | one ROS subscription and one data track per normalized topic |
| ROS video topic | normalized topic name, unique graph type, and matching video topic entry | video track | one in-process video stream per resolved `stream_key` |
| Other video | matching `video.other.*` entry | video track | one in-process video stream per resolved `stream_key` |

### Requirements

- The bridge MUST NOT guess when topic type resolution is ambiguous.
- When the last client lease disappears, the shared data track or video stream MUST be torn down.
- The bridge MUST connect with auto-subscribe disabled and MUST subscribe only to media tracks it names, by exact track name.
- A client publishing a media track the bridge did not subscribe to MUST NOT expect the bridge to receive it.

### Example

Two clients subscribing to the same [normalized](#versioning-and-terminology) non-video ROS topic share one ROS subscription and one data track. The bridge tracks separate leases, but not separate data-track backends.

## Reconnect and Lease Semantics

### Requirements

- Each subscription lease lasts 45 seconds from the heartbeat that renewed it.
- Omitting a target from a later heartbeat MUST NOT cancel its existing lease immediately.
- The bridge MUST allow unrenewed leases to expire naturally.

## Data-Packet Topic: `lkros.heartbeat`

### Purpose

`lkros.heartbeat` requests and renews topic or video subscriptions via a LiveKit data-packet topic, sent from the client to the bridge.

### Example

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
      "kind": "other_video",
      "name": "front_camera"
    },
    {
      "kind": "other_audio",
      "name": "cab_mic"
    }
  ]
}
```

### Request Requirements

#### Validation

- `subscriptions` MUST be present and MUST be an array.
- Each entry MUST be an object with a string `kind` field and, for recognized kinds, a string `name` field.
- `kind` MUST be `topic`, `other_video`, or `other_audio`. When an entry's `kind` is not one of these values, the bridge MUST answer that entry with an [`unsupported_kind`](#lkrosstatus-error-reasons) error on `lkros.status` rather than reject the heartbeat; validation of the remaining entries is unchanged.
- An unrecognized-kind entry is answered as an error in its entirety; its remaining fields are never validated, since the field semantics of an unknown kind cannot be assumed. The error entry echoes the entry's `kind` verbatim and includes `name` only when the client sent a string `name`, echoed verbatim. A missing, non-string, or blank `kind` is a malformed entry and still rejects the heartbeat.
- `topic` names MUST [normalize](#versioning-and-terminology) to non-empty [ROS resource names](#versioning-and-terminology).
- `other_video` names MUST address configured entries from `video.other.<id>`.
- `other_audio` names MUST address configured entries from `audio.other.<id>`.
- For recognized kinds, `delivery_preferences`, when present, MUST be an object.
- For recognized kinds, `delivery_preferences.interval_ms`, when present, MUST be an integer.

#### Authorization

- `topic` subscriptions MUST be authorized against `access.rules.subscribe.*`.
- `other_video` targets MUST NOT use `access.rules.subscribe.*`; they are controlled by the configured `video_other_ids` and `video.other.*` entries.
- `other_audio` targets MUST NOT use `access.rules.subscribe.*`; they are controlled by the configured `audio_other_ids` and `audio.other.*` entries.

#### Coalescing

- `interval_ms` values outside the bridge's native integer range MUST be clamped to that range before duplicate coalescing.
- `interval_ms: 0` means no preference and MUST NOT override a non-zero interval during coalescing.
- Duplicate targets (same [canonical name](#versioning-and-terminology)) MUST coalesce into one effective request in first-seen order, and MUST keep the smallest non-zero `interval_ms`.
- Negative `interval_ms` values MUST remain eligible during coalescing and MUST clamp to `0` only when the lease is applied.

#### Lease Renewal

- Each heartbeat MUST renew the listed subscription leases for 45 seconds.
- Omitting a previously requested target MUST leave its existing lease active until expiry.
- A heartbeat with an empty `subscriptions` array MUST renew nothing and MUST publish no status packet.

#### Client Identity and `session_id`

LiveKit exposes client identity through `caller_identity` on RPCs and `requester_identity` on data packets. Heartbeats prefer `requester_identity`. `session_id` is a fallback for when LiveKit omits that identity from user-data packets.

- `session_id` is optional; missing, `null`, and blank values MUST be treated as absent.
- Heartbeats with a non-empty `requester_identity` MUST be accepted normally.
- If that heartbeat also carries `session_id`, the bridge MUST bind that `session_id` to the client for 45 seconds.
- A later heartbeat with an empty `requester_identity` MUST be accepted only if it carries a known, unexpired `session_id`.
- A `session_id` MUST NOT be rebound to a different client until the existing lease expires.
- Anonymous heartbeats without a known `session_id` MUST be dropped.

## Data-Packet Topic: `lkros.status`

### Purpose

`lkros.status` reports the outcome of a non-empty heartbeat via a LiveKit data-packet topic, sent from the bridge to the client.

### Example

```json
{
  "v": 2,
  "type": "lkros.status",
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
        "track_name": "lkros.data.battery_state",
        "content_type": "application/x-ros-cdr",
        "interval_ms": 100
      }
    }
  ]
}
```

### Envelope Requirements

- `v` MUST be the protocol version, currently `2`.
- `type` MUST always be `lkros.status`.
- `subscriptions` MUST be present on every status packet and MUST be non-empty.
- `subscriptions` MUST reflect the heartbeat's effective request set after canonicalization and coalescing, in effective-request order. Unrecognized-kind error entries are the exception: they MUST appear after recognized targets, in first-seen order.
- `session_id` MUST be included only when the heartbeat carried a non-blank `session_id`.
- `lease_expires_in_ms` MUST be included on every non-empty status packet.
- `lease_expires_in_ms` MUST be treated as approximate; the bridge computes it at serialization time.
- The bridge MUST publish no status packet when the heartbeat produced an empty `subscriptions` array.

### Entry Requirements

#### Shared Fields

Every entry MUST include:

- `kind`.
- `status`: `active` or `error`.
- `name`, except as noted for [unrecognized kinds](#error-entries).

Recognized kinds MUST set `kind` to `topic`, `other_video`, or `other_audio`.

#### Active Entries

Active entries (`status: "active"`):

- MUST include `delivery`.
- MUST include `interface_type` when `kind` is `topic`, even when delivered as video.
- MUST NOT include `interface_type` when `kind` is `other_video` or `other_audio`.
- MUST set `delivery.kind` to `data`, `video`, or `audio`.
- MUST include `delivery.track_name`.
- MAY include `degraded_reason` on video entries when the stream is degraded but still deliverable.
- MAY include `qos` on data-delivered topics: an object with `durability` (`"volatile"` | `"transient_local"`). A `transient_local` topic supports [`ros2.topic.echo.once`](#rpc-ros2topicechoonce).

#### Error Entries

Error entries (`status: "error"`) MUST include:

- `error`.
- `error.reason` (see [`lkros.status` error reasons](#lkrosstatus-error-reasons)).
- `error.message`.

`unsupported_kind` entries additionally:

- MUST set `kind` to the client's unrecognized kind string, echoed verbatim.
- MUST include `name` when the client sent a string `name`, echoed verbatim; otherwise `name` MUST be omitted.
- MUST coalesce repeated identical entries (same echoed `kind` and `name`) into one entry, in first-seen order.

##### Example

```json
{
  "kind": "topic",
  "name": "/restricted_topic",
  "status": "error",
  "error": {
    "reason": "forbidden",
    "message": "subscription denied by policy"
  }
}
```

An `unsupported_kind` entry looks like:

```json
{
  "kind": "hologram_feed",
  "name": "deck_left",
  "status": "error",
  "error": {
    "reason": "unsupported_kind",
    "message": "This bridge does not support subscription kind 'hologram_feed'."
  }
}
```

### Data-Track Delivery

Non-video ROS topics are delivered on a LiveKit data track.

#### Example

```json
{
  "kind": "topic",
  "name": "/battery_state",
  "status": "active",
  "interface_type": "sensor_msgs/msg/BatteryState",
  "qos": {
    "durability": "volatile"
  },
  "delivery": {
    "kind": "data",
    "track_name": "lkros.data.battery_state",
    "content_type": "application/x-ros-cdr",
    "interval_ms": 100
  }
}
```

#### Requirements

- `delivery.kind` MUST be `data`.
- `delivery.track_name` MUST be deterministic: the bridge prefixes `lkros.data` and replaces `/` with `.`.
- `delivery.content_type` MUST be `application/x-ros-cdr`.
- `delivery.interval_ms` MUST always be present for data deliveries, including `0`.
- Bytes on the data track MUST be raw serialized CDR, not nested JSON.

### Video-Track Delivery

Video deliveries use deterministic track names.

#### Example

```json
{
  "kind": "other_video",
  "name": "front_camera",
  "status": "active",
  "delivery": {
    "kind": "video",
    "track_name": "lkros.video.other.front_camera"
  }
}
```

#### Requirements

- `delivery.kind` MUST be `video`.
- `delivery.track_name` MUST always be present.
- `other_video` targets MUST always use video delivery.
- ROS topics MUST use video delivery only when their resolved type is `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage`.
- Active `topic` entries using video delivery MUST still include `interface_type`.
- Video `track_name` values MUST be deterministic and stable for the target name.
- `other_video` track names MUST percent-encode any byte outside the RFC 3986 unreserved set.

### Audio-Track Delivery

Audio deliveries use deterministic track names.

#### Example

```json
{
  "kind": "other_audio",
  "name": "cab_mic",
  "status": "active",
  "delivery": {
    "kind": "audio",
    "track_name": "lkros.audio.other.cab_mic"
  }
}
```

#### Requirements

- `delivery.kind` MUST be `audio`.
- `delivery.track_name` MUST always be present.
- `other_audio` targets MUST always use audio delivery.
- Audio `track_name` values MUST be deterministic and stable for the target name.
- `other_audio` track names MUST percent-encode any byte outside the RFC 3986 unreserved set.
- Each configured other-audio source publishes as exactly one mono audio track; stereo is client-side routing of two mono tracks.

## Remote Media Track: `lkros.audio.out` (Audio Output)

### Purpose

The audio output track carries live audio from a client into the bridge, which plays it through its configured sink. It is the only remote media track the bridge subscribes to, and it exists only on bridges configured with an output sink (see `audio.out.sink` in the [configuration guide](configuration.md#audio-output)). Who may publish is enforced by the application layer, not the bridge: the bridge performs no identity checks on the publisher.

### Name

`lkros.audio.out` is fixed, and bridges advertise it as `features.audio.out.track_name` in [`lkros.capability`](#rpc-lkroscapability). It carries no per-identity suffix — client identities churn on every page reload, and the name describes the track's role: audio the bridge plays out.

### Requirements

- The bridge MUST connect with auto-subscribe disabled and subscribe only to remote tracks it names. Today that is exactly one name: `lkros.audio.out`.
- A client that holds the publish right (granted by the app layer on a verified lease) publishes one audio track with this name; the bridge subscribes to it by exact name and plays the decoded audio out of its configured output.
- The bridge MUST NOT perform identity checks on the publisher; enforcement of who may publish belongs entirely to the app layer.
- Mute is silence-through: a muted publisher MUST keep the track alive with silent frames; the bridge MUST NOT react to mute state.
- When the track is unpublished, the client disconnects, or the client loses its lease (which the app layer signals by unpublishing), the bridge MUST release the sink and rebind it to the next track with this name that delivers a frame.
- A second track with this name delivering frames while one is live MUST be logged and dropped; the active track never loses the sink to a racing publisher.
- A client MUST NOT expect acknowledgement or status entries for audio output; the feature is one-way media with no control-plane messages.
- On a bridge without `audio.out.sink` configured, no track named `lkros.audio.out` is ever subscribed; publishing one is harmless and produces no effect.

### Format

Frames arrive at 10 ms cadence, 48 kHz mono int16 PCM. The bridge reads the first frame's actual rate and channel count and configures its playback pipeline from that; clients SHOULD publish mono 48 kHz.

### Notes

- A client offers audio output only when [`lkros.capability`](#rpc-lkroscapability) advertises `audio.out`; publishing the track without that advertisement has no effect on unconfigured bridges.
- The bridge plays the newest audio: it holds at most 200 ms of received audio and drops the oldest beyond that, so neither a stalled link nor a slow output builds up unbounded delay.

## Byte Stream: `lkros.echo.once`

### Purpose

`lkros.echo.once` delivers the cached last message of a `transient_local` ROS topic to a client that explicitly requests it through the [`ros2.topic.echo.once`](#rpc-ros2topicechoonce) RPC. The bridge responds with a targeted LiveKit byte stream carrying the cached CDR, then reports the outcome in the RPC response.

### Name

`lkros.echo.once` is a single fixed stream topic; it is NOT derived per ROS topic. Each delivery carries the resolved ROS topic name in the byte stream's `name` field, so one client-side handler routes every echo-once delivery. This differs from the live data track, whose name is derived per topic (`lkros.data.<topic>`).

### Payload

- The payload is raw serialized CDR bytes — the same bytes the client would receive on the live data track.
- The content type MUST be `application/x-ros-cdr`.
- There is no JSON or base64 envelope.

### Requirements

- The bridge caches a topic's value only while an active subscription exists for it; `ros2.topic.echo.once` reads that cache and does not itself create a subscription. If no client has an active subscription for the topic, there is no cached value.
- The bridge MUST send a stream only for topics whose resolved subscription durability is `transient_local`; volatile topics are never cached and MUST NOT be sent.
- The bridge MUST send a stream only when a cached value exists for the topic.
- The stream MUST be targeted to exactly the requesting client identity; existing subscribers MUST NOT receive it.
- The subscribe access policy applies exactly as for live delivery; the requesting identity's `access.rules.subscribe.*` is re-checked on every call.
- The byte-stream dispatch is non-blocking: the bridge hands the cached bytes to an asynchronous sender and reports `"sent"` at that point (see [`ros2.topic.echo.once`](#rpc-ros2topicechoonce)). A transfer failure that occurs after the handoff is logged by the bridge but does not change the already-returned `"sent"` result.

### Notes

An echo-once stream MAY arrive after newer live track frames. Clients SHOULD prefer the newer live data when both are received. The payload carries no bridge-applied sequence number; clients that need strict ordering must compare a timestamp inside the message itself.

## Data-Packet Topic: `ros2.topic.pub`

### Purpose

`ros2.topic.pub` is a best-effort write path for small, allowed ROS topic publications, sent from the client to the bridge via a LiveKit data-packet topic.

### Example

```json
{
  "topic": "/cmd_vel",
  "interface_type": "geometry_msgs/msg/Twist",
  "message": {
    "content_type": "application/x-ros-cdr",
    "payload_base64": "AAECAw=="
  }
}
```

### Request Requirements

- `topic` MUST be present, MUST be a string, and MUST [normalize](#versioning-and-terminology) to a non-empty ROS topic name.
- `interface_type` MUST be present, MUST trim to a non-empty string, and MUST exactly match the bridge's resolved topic type.
- `message` MUST be present and MUST decode to a non-empty CDR payload using the [ROS Payload Envelope](#ros-payload-envelope).
- Anonymous packets MUST be dropped.
- The bridge MUST check authorization against `access.rules.publish.*` using the normalized topic name.
- Once a publisher is cached for a topic, later requests MUST be checked against the cached type rather than the ROS graph.
- This path MUST be best-effort and MUST NOT emit an acknowledgement packet.
- Malformed, forbidden, or late requests MUST be logged and dropped (see [Error Model](#error-model)).

### Notes

This path targets a ROS publisher, not the bridge control plane. It is intended for command-style writes, not a high-volume transport over data-packet topics.

## RPC: `ros2.interface.show`

### Purpose

`ros2.interface.show` returns interface definitions a client needs to encode or decode ROS payloads.

### Example Request

```json
{
  "interface_types": [
    "sensor_msgs/msg/Image",
    "std_srvs/srv/Trigger"
  ]
}
```

### Example Response

```json
{
  "interfaces": [
    {
      "interface_type": "sensor_msgs/msg/Image",
      "format": "ros2msg",
      "definition": "..."
    }
  ]
}
```

### Request Requirements

- `interface_types` MUST be present and MUST be a non-empty array.
- Every entry MUST trim to a non-empty string.
- Anonymous calls MUST be rejected (see [Error Model](#error-model)).

### Response Requirements

- A successful response MUST be a JSON object with an `interfaces` array.
- Each entry MUST include `interface_type`, `format`, and `definition`.
- `format` MUST currently be `ros2msg`.
- `definition` MUST be the raw `.msg`, `.srv`, or `.action` file content from the package share directory.
- For each requested type in request order, the bridge MUST append that requested definition first, then any transitive message dependencies in first-discovery order.
- Repeated requested types and shared dependencies MUST appear only once, preserving first-seen response order.

### Notes

This method lets a client obtain message and service definitions before serializing or deserializing CDR payloads. Despite the singular command-style name, the request is batch-oriented so clients can fetch multiple definitions in one round-trip.

## RPC: `ros2.service.call`

### Purpose

`ros2.service.call` performs an authorized ROS request-response operation.

### Example Request

```json
{
  "service": "/my_service",
  "interface_type": "std_srvs/srv/Trigger",
  "request": {
    "content_type": "application/x-ros-cdr",
    "payload_base64": "AAECAw=="
  },
  "timeout_ms": 1000
}
```

### Example Response

```json
{
  "service": "/my_service",
  "interface_type": "std_srvs/srv/Trigger",
  "response": {
    "content_type": "application/x-ros-cdr",
    "payload_base64": "AAECAw=="
  }
}
```

### Request Requirements

- `service` and `request` MUST be present.
- `service` MUST [normalize](#versioning-and-terminology) to a non-empty ROS service name.
- `request.payload_base64` MUST decode to a non-empty byte vector.
- `interface_type` is optional; if omitted or blank, the bridge MUST require exactly one graph-advertised service type.
- `timeout_ms`, when present, MUST be an integer.
- Values `<= 0` MUST NOT disable timeouts; they MUST fall back to the bridge default of `2000` ms.
- Anonymous calls MUST be rejected (see [Error Model](#error-model)).
- The bridge MUST check authorization against `access.rules.service.*` after request parsing and before issuing the ROS request.
- Each `caller_identity` MUST be limited to at most `4` in-flight service calls.
- Some failures MAY happen after acceptance: timeout, client disconnect, session reset, or shutdown.

### Response Requirements

- A successful response MUST be a JSON object with `service`, `interface_type`, and `response`.
- `service` MUST be the [normalized](#versioning-and-terminology) ROS service name the bridge actually invoked.
- `interface_type` MUST be the exact interface type the bridge used.
- `response` MUST use the [ROS Payload Envelope](#ros-payload-envelope).

### Notes

Clients that omit `interface_type` should be prepared for ambiguity to fail the call.

## RPC: `ros2.service.list`

### Purpose

`ros2.service.list` lists authorized ROS services.

### Example Request

```json
{
  "query": "trigger",
  "limit": 10
}
```

### Example Response

```json
{
  "services": [
    {
      "service": "/my_service",
      "interface_type": "std_srvs/srv/Trigger"
    }
  ]
}
```

### Request Requirements

- `query` is optional; missing, `null`, and blank values MUST be treated as absent.
- `limit`, when present, MUST be a positive integer.
- Anonymous calls MUST be rejected (see [Error Model](#error-model)).
- Access checks MUST use `access.rules.service.*`.
- Filtering MUST happen after the ROS graph query and after access-policy checks.
- `query` MUST match substrings in either the resource name or the interface type.
- Resources with zero or multiple interface types MUST be skipped, not returned ambiguously.

### Response Requirements

- A successful response MUST be a JSON object with a `services` array.
- Each entry MUST include `service` and `interface_type`.
- `services` MAY be empty when no authorized resource matches.

## RPC: `ros2.topic.list`

### Purpose

`ros2.topic.list` lists authorized ROS topics.

### Example Request

```json
{
  "query": "image",
  "limit": 10
}
```

### Example Response

```json
{
  "topics": [
    {
      "topic": "/camera/image_raw",
      "interface_type": "sensor_msgs/msg/Image"
    }
  ]
}
```

### Request Requirements

- `query` is optional; missing, `null`, and blank values MUST be treated as absent.
- `limit`, when present, MUST be a positive integer.
- Anonymous calls MUST be rejected (see [Error Model](#error-model)).
- Access checks MUST use `access.rules.subscribe.*`.
- Filtering MUST happen after the ROS graph query and after access-policy checks.
- `query` MUST match substrings in either the resource name or the interface type.
- Resources with zero or multiple interface types MUST be skipped, not returned ambiguously.

### Response Requirements

- A successful response MUST be a JSON object with a `topics` array.
- Each entry MUST include `topic` and `interface_type`.
- `topics` MAY be empty when no authorized resource matches.

## RPC: `ros2.topic.echo.once`

### Purpose

`ros2.topic.echo.once` requests the cached last message of a topic. When a value is available, the bridge delivers it to the caller as a targeted [`lkros.echo.once`](#byte-stream-lkrosechoonce) byte stream and answers the RPC with `{"result": "sent"}`. This lets a late-joining client pull the last message of a `transient_local` topic without waiting for the next publication.

### Example Request

```json
{
  "kind": "topic",
  "name": "/map"
}
```

### Example Response

```json
{
  "result": "sent"
}
```

### Request Requirements

- `kind` MUST be present and MUST be the string `"topic"`; any other value (including subscription kinds such as `other_video` and any future kind) MUST be rejected as an invalid request rather than answered with `"none"`.
- `name` MUST be a present, non-empty string and MUST [normalize](#versioning-and-terminology) to a valid [ROS resource name](#versioning-and-terminology); relative names are expanded to absolute form.
- `kind` and `name` MAY carry surrounding whitespace, which the bridge trims.
- Anonymous calls MUST be rejected (see [Error Model](#error-model)).
- Access checks MUST use `access.rules.subscribe.*` against the requesting identity. A denied topic MUST produce a forbidden error and MUST NOT send a byte stream.

### Response Requirements

- A successful response MUST be a JSON object with a string `result` field whose value is `"sent"` or `"none"`.
- `result` MUST be `"sent"` when a cached value existed and was handed to the byte-stream sender, targeted to the calling identity.
- `result` MUST be `"none"` in every other deliverable case: no active subscription for the topic, an empty cache, a volatile topic, a topic delivered as video, or a synchronous dispatch failure. No byte stream is sent in these cases.
- The bridge intentionally does not distinguish "no value yet" from "never has a value". Clients SHOULD retry a few times on `"none"` (the last message may still be in flight to the bridge) and then stop.

## RPC: `lkros.capability`

### Purpose

`lkros.capability` discloses which optional bridge features are available. It lets a client distinguish a bridge that predates capability discovery (which answers the RPC unknown-method error) from a bridge that knows the RPC and has no features configured (which answers an empty `features` object).

### Example Request

Any payload is valid. The bridge accepts and ignores it:

```json
{}
```

### Example Response

A bridge with no optional features configured:

```json
{
  "v": 2,
  "features": {}
}
```

A bridge with an audio output sink configured:

```json
{
  "v": 2,
  "features": {
    "audio": {
      "out": {
        "track_name": "lkros.audio.out"
      }
    }
  }
}
```

### Request Requirements

- The request payload MUST be accepted regardless of content; the bridge MUST ignore it.
- Calls MUST NOT require `caller_identity`; anonymous calls MUST be answered.
- Access policy MUST NOT apply: this RPC MUST be answerable regardless of `access.rules.*`.

### Response Requirements

- A successful response MUST be a JSON object with a `features` field.
- `v` MUST be the protocol version, currently `2`.
- `features` MUST be a JSON object keyed by feature name. Presence of a key advertises the feature, and its value MUST be a JSON object carrying the feature's details (empty when it has none). Related features MAY be grouped under one key, as `audio.out` is under `audio`; a group with no available feature MUST be absent.
- A feature that is not available on the bridge MUST be absent from the object rather than advertised with `false` or `null`.
- A feature MUST be advertised if and only if its availability is configuration-derived; a feature absent from `features` means "not available on this bridge".
- `audio.out` MUST be advertised if and only if the bridge has an output sink configured (`audio.out.sink`); an advertised `audio.out` means the bridge subscribes to the audio output track (see [Remote Media Track: `lkros.audio.out`](#remote-media-track-lkrosaudioout-audio-output)) and plays it.
- `audio.out.track_name` MUST be the name of that track. A client MUST publish under the advertised `track_name`.
- The schema is additive: new feature names and new fields MAY appear in later versions, and clients MUST ignore unknown feature names and unknown fields.
- A bridge that predates this RPC answers with the LiveKit SDK's built-in unsupported-method error (`1400`). A client MUST treat that error as "this bridge does not support capability discovery" and MUST NOT interpret it as an empty feature set; the error and the empty `features` object are the two states of the discovery contract.

### Notes

Any LiveKit room participant MAY call this method at any time — typically on room join — without a heartbeat flow or subscription state. On bridges that predate this RPC, the LiveKit SDK answers with its built-in unsupported-method error (`1400`), the error side of the two-state contract above.

The response is delivered unicast to the caller, who may be any room participant, and contains no ROS resource names, so it discloses nothing about the ROS graph.

## Informative: `ros2` CLI Mapping

Rough mapping from familiar `ros2` commands to the bridge. Request-response work uses RPCs, one-shot topic writes use data-packet topics, and streams arrive on data or video tracks. `lkros.heartbeat` and `lkros.status` are bridge control messages, not ROS messages.

| ROS 2 | Bridge | Comments |
| --- | --- | --- |
| `ros2 action *` | No equivalent | Not supported. |
| `ros2 interface show <type>` | RPC `ros2.interface.show` | Returns the raw ROS interface definition. Batch types via `interface_types`. |
| `ros2 param *` | No equivalent | Not supported. |
| `ros2 service call /service Type ...` | RPC `ros2.service.call` | Uses the shared JSON CDR envelope with base64 bytes, not ROS CLI text. |
| `ros2 service list` | RPC `ros2.service.list` | Lists services this client may call, with interface types. |
| `ros2 topic echo /topic` | `lkros.heartbeat` → `lkros.status` → data or video track | Send a heartbeat, read the status, then read the named track. Most topics use a data track; `sensor_msgs/msg/Image` and `sensor_msgs/msg/CompressedImage` may use a video track. |
| `ros2 topic list` | RPC `ros2.topic.list` | Lists topics this client may use, with interface types. |
| `ros2 topic echo --once /topic` | RPC `ros2.topic.echo.once` → `lkros.echo.once` byte stream | Pulls the cached last message of a `transient_local` topic. Requires an active subscription to populate the cache; returns `none` for volatile, video, or uncached topics. |
| `ros2 topic pub /topic Type ...` | Data-packet topic `ros2.topic.pub` | Best-effort single-message publish for small allowed writes. No ack. |

## Informative: Walkthroughs

### Typical Client Flow

Most integrations follow this order:

1. Join the same LiveKit room as the bridge.
2. Call `lkros.capability` to learn which optional bridge features are available (an unsupported-method error means the bridge predates feature discovery).
3. Call `ros2.topic.list` and `ros2.service.list` to discover the resources your policy allows.
4. Call `ros2.interface.show` for the message and service types you need to encode or decode.
5. Use `ros2.service.call` for request-response operations.
6. Send `ros2.topic.pub` packets for small allowed topic writes.
7. Send `lkros.heartbeat` on a regular cadence to request topic or video subscriptions.
8. Read `lkros.status` to learn whether each requested subscription is active, forbidden, or not found.
9. Subscribe to the announced LiveKit data track or video publication.

For a first integration, start with one service-call path or one topic-subscription path. Once that works, add more interface types, video, and broader policy rules.

### Service Call Flow

A common request-response path:

1. Call `ros2.service.list` to discover an allowed service.
2. Call `ros2.interface.show` for the service type.
3. Serialize the request payload as ROS CDR.
4. Call `ros2.service.call`.
5. Decode the returned ROS CDR response payload.

### Topic Subscription Flow

A common data-subscription path:

1. Send `lkros.heartbeat` with a `topic` subscription request.
2. Read `lkros.status`.
3. If the status is `active` and `delivery.kind` is `data`, subscribe to the announced LiveKit data track.
4. Decode incoming bytes on that track as raw ROS CDR for the reported `interface_type`.

### Other-Video Flow

A common non-ROS video path:

1. Send `lkros.heartbeat` with `kind: "other_video"` and the configured source id as `name`.
2. Read `lkros.status`.
3. If the status is `active` and `delivery.kind` is `video`, subscribe to the announced LiveKit video publication.

### Other-Audio Flow

A common non-ROS audio path:

1. Send `lkros.heartbeat` with `kind: "other_audio"` and the configured source id as `name`.
2. Read `lkros.status`.
3. If the status is `active` and `delivery.kind` is `audio`, subscribe to the announced LiveKit audio publication.
4. Route each mono audio track to a speaker (e.g. left/right) for a stereo-operator experience.

### Audio Output Flow

A common audio output path (only on bridges advertising `audio.out`):

1. Call `lkros.capability` on join; offer audio output only when `features.audio.out` is present, and read its `track_name`.
2. When output starts, publish one audio track named with the advertised `track_name` (48 kHz mono).
3. Keep publishing while output is live; a mute is silence-through, and no further signaling is needed.
4. On lease loss, unpublish the track so the next lease holder can claim the bridge's sink.
5. Regaining the lease republishes when output next starts; the bridge rebinds its sink to the track's first frame.

### Heartbeat with `session_id` Fallback

A common pattern: a browser tab sends a heartbeat with both `requester_identity` and `session_id`, then continues using the same `session_id` if later packets lack identity. The lease expires after 45 seconds, so the client must keep heartbeating before then.

### Reconnect After Page Refresh

If a browser refreshes but sends a valid heartbeat before the old lease expires, the bridge can re-announce the same data-track delivery without recreating the subscription.

## Informative: References

LiveKit concepts this specification builds on:

- [Rooms, participants, and tracks](https://docs.livekit.io/intro/basics/rooms-participants-tracks/) — the primitives every LiveKit client inherits.
- [Data packets](https://docs.livekit.io/home/client/data/packets/) — the `publishData` API, the `topic` field, delivery modes, and size limits that underlie `lkros.heartbeat`, `lkros.status`, and `ros2.topic.pub`.
- [Remote method calls (RPC)](https://docs.livekit.io/home/client/data/rpc/) — the `performRpc` / `registerRpcMethod` pair and the `callerIdentity` available to handlers. LiveKit reserves RPC error codes `1001`–`1999` for its own use; the bridge's `2400`-series codes live above that range.

ROS 2 concepts that clarify payload semantics:

- [About ROS 2 interfaces](https://docs.ros.org/en/rolling/Concepts/Basic/About-Interfaces.html) — the `.msg`, `.srv`, and `.action` file format returned by `ros2.interface.show` and used to encode CDR payloads.
