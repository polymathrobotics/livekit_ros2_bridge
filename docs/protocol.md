# Client Interface and Protocol Specification

This document specifies the client-facing interface and protocol of `livekit_ros2_bridge`.

Unless labeled informative, every section and subsection is normative. Examples, notes, and the final client-flow walkthrough are informative. `MUST`, `MUST NOT`, `SHOULD`, and `MAY` carry their usual normative meaning.

## Purpose, Scope, and Conformance

This specification defines the external contract between a client and the bridge:

- LiveKit RPC methods exposed by the bridge
- LiveKit data-packet topics accepted by the bridge
- LiveKit data-packet topic messages and media deliveries emitted by the bridge
- shared request, response, authorization, and delivery semantics

A conforming client MUST follow the schemas and behavioral rules below, and MUST NOT rely on any behavior this document does not specify.

## Versioning and Terminology

The `lkros.status` packet carries a protocol version field `v`, currently `2`.

Terms used throughout:

- bridge: the `livekit_ros2_bridge` participant in the LiveKit room
- client: the non-bridge participant interacting with the bridge over LiveKit
- data-packet topic: the LiveKit topic string on a `publishData` packet (outside RPC)
- control-plane message: a bridge-directed message about subscription lease or status state
- ROS publish request: a bridge-accepted request to publish one message to a ROS topic
- data track: a LiveKit data publication carrying raw ROS CDR bytes
- video track: a LiveKit video publication carrying a ROS-backed or GStreamer-backed stream
- normalized name: a ROS name converted to absolute form before validation and policy checks
- lease: the bridge's time-bounded record that a client still wants a subscription

## Protocol Surfaces

Every surface in this specification runs over LiveKit. Requests and control flows use RPCs or data packets; streams use data or video tracks.

| Type | Name | Flow | Comments |
| --- | --- | --- | --- |
| Data-Packet Topic | `lkros.heartbeat` | client -> bridge | Request and renew subscriptions |
| Data-Packet Topic | `lkros.status` | bridge -> client | Report per-subscription status |
| Data-Packet Topic | `ros2.topic.pub` | client -> bridge | Best-effort ROS topic publication |
| Data Track | `delivery.track_name` | bridge -> client | Streams raw ROS CDR bytes for active non-video topics. Clients learn the track name from an active `lkros.status` entry with `delivery.kind: "data"`. |
| Video Track | `delivery.track_name` | bridge -> client | Streams ROS image topics or configured `other_video` sources. Clients learn the track name from an active `lkros.status` entry with `delivery.kind: "video"`. |
| RPC | `ros2.interface.show` | client <-> bridge | Fetch interface definitions |
| RPC | `ros2.service.call` | client <-> bridge | Call an authorized ROS service |
| RPC | `ros2.service.list` | client <-> bridge | List authorized ROS services |
| RPC | `ros2.topic.list` | client <-> bridge | List authorized ROS topics |

### `ros2` Command Mapping (informative)

Rough mapping from familiar `ros2` commands to the bridge. Request-response work uses RPCs, one-shot topic writes use data-packet topics, and streams arrive on data or video tracks. `lkros.heartbeat` and `lkros.status` are bridge control messages, not ROS messages.

| ROS 2 | Bridge | Comments |
| --- | --- | --- |
| `ros2 action *` | No equivalent | Not supported. |
| `ros2 interface show <type>` | RPC `ros2.interface.show` | Returns the raw ROS interface definition. Batch types via `interface_types`. |
| `ros2 param *` | No equivalent | Not supported. |
| `ros2 service call /service Type ...` | RPC `ros2.service.call` | Uses the shared JSON CDR envelope with base64 bytes, not ROS CLI text. |
| `ros2 service list` | RPC `ros2.service.list` | Lists services this client may call, with interface types. |
| `ros2 topic echo /topic` | `lkros.heartbeat` -> `lkros.status` -> data or video track | Send a heartbeat, read the status, then read the named track. Most topics use a data track; `sensor_msgs/msg/Image` and `sensor_msgs/msg/CompressedImage` may use a video track. |
| `ros2 topic list` | RPC `ros2.topic.list` | Lists topics this client may use, with interface types. |
| `ros2 topic pub /topic Type ...` | Data-packet topic `ros2.topic.pub` | Best-effort single-message publish for small allowed writes. No ack. |

## Shared Wire Rules

Any ROS binary payload inside a JSON request or response MUST use this envelope:

```json
{
  "content_type": "application/x-ros-cdr",
  "payload_base64": "AAECAw=="
}
```

Requirements:

- `content_type` MUST be `application/x-ros-cdr`
- `payload_base64` MUST be padded standard base64
- wrong `content_type`, wrong field types, or invalid base64 MUST be rejected
- parsers MUST trim string fields before validation
- blank optional strings SHOULD be treated as absent unless an operation says otherwise
- unknown JSON fields SHOULD be ignored unless an operation says otherwise
- whether a decoded payload may be empty is operation-specific; operations that require a non-empty payload say so
- this envelope does not apply to data-track frames, which carry raw serialized CDR bytes

### Example (informative)

The same envelope is used in:

- `ros2.topic.pub.message`
- `ros2.service.call.request`
- `ros2.service.call.response`

## Identity, Authorization, and Name Normalization

Requirements:

- ROS names MUST be normalized to absolute form before validation and policy checks
- if normalization produces an empty name, the request MUST be treated as invalid
- anonymous RPC calls MUST be rejected
- anonymous `ros2.topic.pub` packets MUST be dropped
- anonymous `lkros.heartbeat` packets MUST be accepted only via the `session_id` fallback defined below
- `ros2.topic.pub` MUST check authorization against `access.rules.publish.*`
- `ros2.service.call` and `ros2.service.list` MUST check authorization against `access.rules.service.*`
- topic subscriptions and `ros2.topic.list` MUST check authorization against `access.rules.subscribe.*`
- `other_video` targets MUST NOT use `access.rules.subscribe.*`; they are controlled by the configured `video_other_ids` and `video.other.*` entries

### Example (informative)

If a client asks to publish `cmd_vel`, the bridge normalizes that name first, then validates and checks publish policy. An empty normalized name makes the request invalid. A valid name that fails policy causes the packet to be dropped.

## Data-Packet Topic: `lkros.heartbeat`

`lkros.heartbeat` requests and renews topic or video subscriptions via a LiveKit data-packet topic.

### Example Request (informative)

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
    }
  ]
}
```

### Requirements

- `subscriptions` MUST be present and MUST be an array
- each entry MUST be an object with string `kind` and `name` fields
- `kind` MUST be `topic` or `other_video`
- `topic` names MUST normalize as ROS resource names
- `other_video` names MUST address configured entries from `video.other.<id>`
- `delivery_preferences`, when present, MUST be an object
- `delivery_preferences.interval_ms`, when present, MUST be an integer
- `interval_ms` values outside the bridge's native integer range MUST be clamped to that range before duplicate coalescing
- `interval_ms: 0` means no preference and MUST NOT override a non-zero interval during coalescing
- duplicate targets (same canonical `(kind, name)` after topic normalization or other-video name trimming) MUST coalesce into one effective request in first-seen order, and MUST keep the smallest non-zero `interval_ms`
- negative `interval_ms` values MUST remain eligible during coalescing and MUST clamp to `0` only when the lease is applied
- `session_id` is optional; missing, `null`, and blank values MUST be treated as absent
- each heartbeat MUST renew the listed subscription leases for 45 seconds
- omitting a previously requested target MUST leave its existing lease active until expiry
- a heartbeat with an empty `subscriptions` array MUST renew nothing and MUST publish no status packet

### Client Identity and `session_id`

LiveKit exposes client identity through `caller_identity` on RPCs and `requester_identity` on data packets. Heartbeats prefer `requester_identity`. `session_id` is a fallback for when LiveKit omits that identity from user-data packets.

Requirements:

- heartbeats with a non-empty `requester_identity` MUST be accepted normally
- if that heartbeat also carries `session_id`, the bridge MUST bind that `session_id` to the client for 45 seconds
- a later heartbeat with an empty `requester_identity` MUST be accepted only if it carries a known, unexpired `session_id`
- a `session_id` MUST NOT be rebound to a different client until the existing lease expires
- anonymous heartbeats without a known `session_id` MUST be dropped

### Example (informative)

A common pattern: a browser tab sends a heartbeat with both `requester_identity` and `session_id`, then continues using the same `session_id` if later packets lack identity. The lease expires after 45 seconds, so the client must keep heartbeating before then.

## Data-Packet Topic: `lkros.status`

`lkros.status` reports the outcome of a non-empty heartbeat via a LiveKit data-packet topic.

### Example Response (informative)

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

- `v` MUST be the protocol version, currently `2`
- `type` MUST always be `lkros.status`
- `subscriptions` MUST be present on every status packet and MUST be non-empty
- `subscriptions` MUST reflect the heartbeat's effective request set after canonicalization and coalescing, in effective-request order
- `session_id` MUST be included only when the heartbeat carried a non-blank `session_id`
- `lease_expires_in_ms` MUST be included on every non-empty status packet
- `lease_expires_in_ms` MUST be treated as approximate; the bridge computes it at serialization time
- the bridge MUST publish no status packet when the heartbeat produced an empty `subscriptions` array

### Per-Subscription Status Objects

Active entries MUST include:

- `kind`: `topic` or `other_video`
- `name`
- `status`: `active`
- `delivery`

Error entries MUST include:

- `kind`
- `name`
- `status`: `error`
- `error`
- `error.reason`
- `error.message`

Additional requirements:

- active `topic` entries MUST include `interface_type`, even when delivered as video
- active `other_video` entries MUST NOT include `interface_type`
- active `delivery.kind` MUST be `data` or `video`
- active `delivery.track_name` MUST always be present
- active video entries MAY include `degraded_reason` when the stream is degraded but still deliverable

Current `error.reason` values:

- `forbidden`: subscribe policy denies the topic
- `unavailable`: a required runtime dependency (typically a video pipeline) could not start or keep running
- `not_found`: lookup or subscription creation failed for another reason

### Active Topic Subscriptions on a Data Track

Non-video ROS topics are delivered on a LiveKit data track.

#### Example (informative)

```json
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
```

Requirements:

- `delivery.kind` MUST be `data`
- `delivery.track_name` MUST be deterministic: the bridge prefixes `lkros.data` and replaces `/` with `.`
- `delivery.content_type` MUST be `application/x-ros-cdr`
- `delivery.interval_ms` MUST always be present for data deliveries, including `0`
- bytes on the data track MUST be raw serialized CDR, not nested JSON

### Active Video Subscription Entries

Video deliveries use deterministic track names.

#### Example (informative)

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

Requirements:

- `delivery.kind` MUST be `video`
- `delivery.track_name` MUST always be present
- `other_video` targets MUST always use video delivery
- ROS topics MUST use video delivery only when their resolved type is `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage`
- active `topic` entries using video delivery MUST still include `interface_type`
- video `track_name` values MUST be deterministic and stable for the target name
- `other_video` track names MUST percent-encode any byte outside the RFC 3986 unreserved set

#### Error Example (informative)

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

## Data-Packet Topic: `ros2.topic.pub`

`ros2.topic.pub` is a best-effort write path for small, allowed ROS topic publications, sent via a LiveKit data-packet topic.

### Example Request (informative)

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

### Requirements

- `topic` MUST be present, MUST be a string, and MUST normalize to a non-empty ROS topic name
- `interface_type` MUST be present, MUST trim to a non-empty string, and MUST exactly match the bridge's resolved topic type
- `message` MUST be present and MUST decode to a non-empty CDR payload
- the bridge MUST check publish authorization against the normalized topic name
- once a publisher is cached for a topic, later requests MUST be checked against the cached type rather than the ROS graph
- this path MUST be best-effort and MUST NOT emit an acknowledgement packet
- malformed, forbidden, or late requests MUST be logged and dropped

### Notes (informative)

This path targets a ROS publisher, not the bridge control plane. It is intended for command-style writes, not a high-volume transport over data-packet topics.

## RPC: `ros2.service.call`

`ros2.service.call` performs an authorized ROS request-response operation.

### Example Request (informative)

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

### Example Successful Response (informative)

```json
{
  "ok": true,
  "service": {
    "name": "/my_service",
    "interface_type": "std_srvs/srv/Trigger"
  },
  "response": {
    "content_type": "application/x-ros-cdr",
    "payload_base64": "AAECAw=="
  },
  "elapsed_ms": 12
}
```

### Request Requirements

- `service` and `request` MUST be present
- `service` MUST normalize to a non-empty ROS service name
- `request.payload_base64` MUST decode to a non-empty byte vector
- `interface_type` is optional; if omitted or blank, the bridge MUST require exactly one graph-advertised service type
- `timeout_ms`, when present, MUST be an integer
- values `<= 0` MUST NOT disable timeouts; they MUST fall back to the bridge default of `2000` ms
- the bridge MUST check access policy after request parsing and before issuing the ROS request
- each `caller_identity` MUST be limited to at most `4` in-flight service calls
- some failures MAY happen after acceptance: timeout, client disconnect, session reset, or shutdown

### Successful Response Requirements

- a successful response MUST be a JSON object with `ok`, `service`, `response`, and `elapsed_ms`
- `ok` MUST be `true`
- `service.name` MUST be the normalized ROS service name the bridge actually invoked
- `service.interface_type` MUST be the exact interface type the bridge used
- `response` MUST use the shared ROS CDR JSON envelope
- `elapsed_ms` MUST be a non-negative integer measured by the bridge

### Notes (informative)

Clients that omit `interface_type` should be prepared for ambiguity to fail the call.

## RPC: `ros2.interface.show`

`ros2.interface.show` returns interface definitions a client needs to encode or decode ROS payloads.

### Example Request (informative)

```json
{
  "interface_types": [
    "sensor_msgs/msg/Image",
    "std_srvs/srv/Trigger"
  ]
}
```

### Example Successful Response (informative)

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

- `interface_types` MUST be present and MUST be a non-empty array
- every entry MUST trim to a non-empty string

### Successful Response Requirements

- a successful response MUST be a JSON object with an `interfaces` array
- each entry MUST include `interface_type`, `format`, and `definition`
- `format` MUST currently be `ros2msg`
- `definition` MUST be the raw `.msg`, `.srv`, or `.action` file content from the package share directory
- for each requested type in request order, the bridge MUST append that requested definition first, then any transitive message dependencies in first-discovery order
- repeated requested types and shared dependencies MUST appear only once, preserving first-seen response order

### Notes (informative)

This method lets a client obtain message and service definitions before serializing or deserializing CDR payloads. Despite the singular command-style name, the request is batch-oriented so clients can fetch multiple definitions in one round-trip.

## RPC: `ros2.service.list`

`ros2.service.list` lists authorized ROS services.

### Example Request (informative)

```json
{
  "query": "trigger",
  "limit": 10
}
```

### Example Successful Response (informative)

```json
{
  "services": [
    {
      "name": "/my_service",
      "interface_type": "std_srvs/srv/Trigger"
    }
  ]
}
```

### Request Requirements

- `query` is optional; missing, `null`, and blank values MUST be treated as absent
- `limit`, when present, MUST be a positive integer
- filtering MUST happen after the ROS graph query and after access-policy checks
- `query` MUST match substrings in either the resource name or the interface type
- resources with zero or multiple interface types MUST be skipped, not returned ambiguously

### Successful Response Requirements

- a successful response MUST be a JSON object with a `services` array
- each entry MUST include `name` and `interface_type`
- `services` MAY be empty when no authorized resource matches

## RPC: `ros2.topic.list`

`ros2.topic.list` lists authorized ROS topics.

### Example Request (informative)

```json
{
  "query": "image",
  "limit": 10
}
```

### Example Successful Response (informative)

```json
{
  "topics": [
    {
      "name": "/camera/image_raw",
      "interface_type": "sensor_msgs/msg/Image"
    }
  ]
}
```

### Request Requirements

- `query` is optional; missing, `null`, and blank values MUST be treated as absent
- `limit`, when present, MUST be a positive integer
- filtering MUST happen after the ROS graph query and after access-policy checks
- `query` MUST match substrings in either the resource name or the interface type
- resources with zero or multiple interface types MUST be skipped, not returned ambiguously

### Successful Response Requirements

- a successful response MUST be a JSON object with a `topics` array
- each entry MUST include `name` and `interface_type`
- `topics` MAY be empty when no authorized resource matches

## Delivery and Sharing Model

| Requested target | Resolved from | Delivery | Shared resource model |
| --- | --- | --- | --- |
| Non-video ROS topic | normalized topic name and unique graph type | data | one ROS subscription and one data track per normalized topic |
| ROS video topic | normalized topic name, unique graph type, and matching video topic entry | video | one in-process video stream per resolved `stream_key` |
| Other video | matching `video.other.*` entry | video | one in-process video stream per resolved `stream_key` |

Requirements:

- topic subscriptions MUST use `access.rules.subscribe.*`
- `other_video` targets MUST NOT use subscribe rules
- the bridge MUST NOT guess when topic type resolution is ambiguous
- when the last client lease disappears, the shared data track or video stream MUST be torn down

### Example (informative)

Two clients subscribing to the same normalized non-video ROS topic share one ROS subscription and one data track. The bridge tracks separate leases, but not separate data-track backends.

## Reconnect and Lease Semantics

Requirements:

- each subscription lease lasts 45 seconds from the heartbeat that renewed it
- omitting a target from a later heartbeat MUST NOT cancel its existing lease immediately
- the bridge MUST allow unrenewed leases to expire naturally
- if a client reconnects while still owning a live data subscription, the bridge MUST republish the data track after a heartbeat confirms the client
- a well-formed heartbeat MAY therefore trigger one data-track republish after a page refresh or participant reconnect

### Example (informative)

If a browser refreshes but sends a valid heartbeat before the old lease expires, the bridge can re-announce the same data-track delivery without recreating the subscription.

## Error Model

Data-packet topics and RPCs fail differently:

- malformed `ros2.topic.pub` packets, unsupported data-packet topics, and anonymous publish requests are logged and dropped
- malformed subscription heartbeats are logged and dropped
- well-formed subscription heartbeats report per-target failures through `lkros.status`
- RPC failures surface through LiveKit RPC errors

Stable RPC error codes:

| Code | Meaning | Typical causes |
| --- | --- | --- |
| `2400` | invalid request | bad JSON, wrong field types, empty canonical names, payload decode failures, invalid bounds |
| `2401` | unauthorized | missing `caller_identity` |
| `2403` | forbidden | access policy denies the requested service or topic |
| `2500` | internal | ROS graph failures, client creation failures, runtime exceptions, late service-call failures |

Input validation errors (bad arguments, out-of-range values) MUST map to `2400`. All other unhandled errors MUST map to `2500` unless the handler has already raised a more specific error.

### Example (informative)

If a client sends malformed `ros2.topic.pub` JSON, the bridge logs and drops it with no reply. If the client sends a valid heartbeat for a forbidden topic, the bridge still replies on `lkros.status`, but that entry has `status: "error"` and reason `forbidden`. If the client calls `ros2.service.call` without `caller_identity`, the call fails with RPC code `2401`.

## Informative Examples

### Service Call Flow (informative)

A common request-response path:

1. Call `ros2.service.list` to discover an allowed service.
2. Call `ros2.interface.show` for the service type.
3. Serialize the request payload as ROS CDR.
4. Call `ros2.service.call`.
5. Decode the returned ROS CDR response payload.

### Topic Subscription Flow (informative)

A common data-subscription path:

1. Send `lkros.heartbeat` with a `topic` subscription request.
2. Read `lkros.status`.
3. If the status is `active` and `delivery.kind` is `data`, subscribe to the announced LiveKit data track.
4. Decode incoming bytes on that track as raw ROS CDR for the reported `interface_type`.

### Other Video Flow (informative)

A common non-ROS video path:

1. Send `lkros.heartbeat` with `kind: "other_video"` and the configured source id as `name`.
2. Read `lkros.status`.
3. If the status is `active` and `delivery.kind` is `video`, subscribe to the announced LiveKit video publication.

## Informative Typical Client Flow

Most integrations follow this order:

1. Join the same LiveKit room as the bridge.
2. Call `ros2.topic.list` and `ros2.service.list` to discover the resources your policy allows.
3. Call `ros2.interface.show` for the message and service types you need to encode or decode.
4. Use `ros2.service.call` for request-response operations.
5. Send `ros2.topic.pub` packets for small allowed topic writes.
6. Send `lkros.heartbeat` on a regular cadence to request topic or video subscriptions.
7. Read `lkros.status` to learn whether each requested subscription is active, forbidden, unavailable, or not found.
8. Subscribe to the announced LiveKit data track or video publication.

For a first integration, start with one service-call path or one topic-subscription path. Once that works, add more interface types, video, and broader policy rules.
