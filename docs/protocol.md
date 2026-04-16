# Client Interface and Protocol Specification

This document is the authoritative specification for the client-facing interface and protocol of `livekit_ros2_bridge`.

Unless a section or subsection is labeled informative, it is normative. Examples, notes, and the final client-flow walkthrough are informative. This document uses `MUST`, `MUST NOT`, `SHOULD`, and `MAY` in their usual normative sense.

## Purpose, Scope, and Conformance

This specification defines the external contract between a client and the bridge:

- LiveKit RPC methods exposed by the bridge
- LiveKit data-packet topics accepted by the bridge
- LiveKit data-packet topic messages and media deliveries emitted by the bridge
- shared request, response, authorization, and delivery semantics

A conforming client MUST follow the request and response schemas and the behavioral rules in this document. Clients MUST NOT rely on behavior that this document does not specify.

## Versioning and Terminology

This document describes the current client-facing protocol of the bridge. The `ros.subscriptions.status` packet includes a protocol version field `v`, which is currently `2`.

Terms used throughout this document:

- bridge: the `livekit_ros2_bridge` participant in the LiveKit room
- caller: the client making an RPC request
- requester: the client sending a message on a LiveKit data-packet topic or owning a subscription lease
- data-packet topic: the LiveKit topic string attached to a `publishData` packet outside the RPC surface
- control-plane message: a bridge-directed message about subscription lease or status state
- ROS publish request: a bridge-accepted request that publishes one message to a ROS topic
- data track: a LiveKit data publication carrying raw ROS CDR bytes
- video track: a LiveKit video publication carrying a ROS-backed or GStreamer-backed stream
- normalized name: a ROS resource name after the bridge has converted it to absolute-style form for validation and policy checks
- lease: the bridge's time-bounded record that a requester still wants a subscription

## Protocol Surfaces

| Surface | Name | Role | Direction | Purpose |
| --- | --- | --- | --- | --- |
| Data-Packet Topic | `ros2.topic.pub` | ROS Publish Request | caller -> bridge | Best-effort ROS topic publication |
| Data-Packet Topic | `ros.subscriptions.heartbeat` | Control-Plane Request | caller -> bridge | Request and renew subscriptions |
| Data-Packet Topic | `ros.subscriptions.status` | Control-Plane Status | bridge -> caller | Per-subscription status |
| RPC | `ros2.service.call` | Request-Response | caller <-> bridge | Call an authorized ROS service |
| RPC | `ros2.interface.show` | Request-Response | caller <-> bridge | Fetch interface definitions |
| RPC | `ros2.service.list` | Request-Response | caller <-> bridge | List authorized ROS services |
| RPC | `ros2.topic.list` | Request-Response | caller <-> bridge | List authorized ROS topics |

The `ros2.*` RPC and data-packet names intentionally mirror the corresponding ROS 2 CLI command
names. Their payloads do not mirror ROS CLI text, flags, or YAML; they remain the bridge's
JSON/CDR protocol defined in this document.

## Shared Wire Rules

ROS binary payloads MUST use the same JSON envelope everywhere they appear:

```json
{
  "content_type": "application/x-ros-cdr",
  "payload_base64": "AAECAw=="
}
```

Requirements:

- `content_type` MUST be `application/x-ros-cdr`
- `payload_base64` MUST be padded standard base64
- wrong `content_type`, wrong JSON field types, and invalid base64 MUST be rejected
- parsers MUST trim string fields before validation
- blank optional strings SHOULD be treated as absent unless an operation says otherwise
- unknown JSON fields SHOULD be ignored unless an operation says otherwise
- whether the decoded payload may be empty is operation-specific; operations that require a non-empty payload say so explicitly

### Example (informative)

The same envelope shape is used for:

- `ros2.topic.pub.message`
- `ros2.service.call.request`
- `ros2.service.call.response`

## Identity, Authorization, and Name Normalization

Requirements:

- ROS names MUST be normalized to absolute-style names before validation and policy checks
- if normalization produces an empty name, the request MUST be treated as invalid
- anonymous RPC calls MUST be rejected up front
- anonymous `ros2.topic.pub` packets MUST be dropped
- anonymous `ros.subscriptions.heartbeat` packets MUST be accepted only through the `session_id` fallback defined in this document
- `ros2.topic.pub` MUST check authorization against `access.rules.publish.*`
- `ros2.service.call` and `ros2.service.list` MUST check authorization against `access.rules.service.*`
- topic subscriptions and `ros2.topic.list` MUST check authorization against `access.rules.subscribe.*`
- `other_video` targets MUST NOT use `access.rules.subscribe.*`; they are controlled by configured `video_other_ids` and `video.other.*` entries

### Example (informative)

If a client asks to publish `cmd_vel`, the bridge normalizes that name before it validates the request or checks publish policy. If the normalized name is empty, the request is invalid. If the normalized name is valid but not allowed by publish policy, the bridge drops the packet.

## Data-Packet Topic: `ros2.topic.pub`

`ros2.topic.pub` is a ROS publish request on a LiveKit data-packet topic. It is a best-effort write path for small allowed ROS topic publications.

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
- `interface_type` MUST be present, MUST trim to a non-empty string, and MUST match the bridge's resolved topic type exactly
- `message` MUST be present and MUST decode to a non-empty CDR payload
- the bridge MUST check publish authorization against the normalized topic name
- once the bridge has cached a publisher for a topic, later requests MUST be checked against that cached type instead of re-reading the ROS graph
- this path MUST be best-effort and MUST NOT emit an acknowledgement packet
- malformed, forbidden, or late requests MUST be logged and dropped

### Notes (informative)

This path targets a ROS publisher, not the bridge control plane. It is intended for command-style writes, not for building a high-volume transport on top of data-packet topics.

## Data-Packet Topic: `ros.subscriptions.heartbeat`

`ros.subscriptions.heartbeat` is a control-plane request on a LiveKit data-packet topic. It requests and renews topic or video subscriptions.

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
- each subscription entry MUST be an object with string `kind` and `name` fields
- `kind` MUST be `topic` or `other_video`
- `topic` names MUST normalize as ROS resource names
- `other_video` names MUST address configured entries from `video.other.<id>`
- `delivery_preferences` MAY be present, but when present it MUST be an object
- `delivery_preferences.interval_ms` MAY be present, but when present it MUST be an integer
- `interval_ms: 0` MUST mean no preference
- if the same target appears more than once, the bridge MUST produce one effective request and MUST use the smallest non-zero `interval_ms`
- negative `interval_ms` values MUST clamp to `0` when the lease is applied
- `session_id` is optional; missing, `null`, and blank values MUST be treated as absent
- each heartbeat MUST renew the listed subscriptions for 45 seconds
- omitting a previously requested target MUST leave its existing lease active until expiry
- a heartbeat with an empty `subscriptions` array MUST renew nothing and MUST publish no status packet

### Requester Identity and `session_id`

The bridge prefers the LiveKit packet's `requester_identity`. `session_id` exists only to keep heartbeats working when LiveKit omits that identity from user-data packets.

Requirements:

- heartbeats with a non-empty `requester_identity` MUST be accepted normally
- if that heartbeat also includes `session_id`, the bridge MUST bind that `session_id` to the requester for 45 seconds
- a later heartbeat with an empty `requester_identity` MUST be accepted only if it includes a known, unexpired `session_id`
- a `session_id` MUST NOT be rebound to a different requester until the existing lease expires
- anonymous heartbeats without a known `session_id` MUST be dropped

### Example (informative)

One practical pattern is for a browser tab to send a heartbeat with both `requester_identity` and `session_id`, then continue using the same `session_id` if later user-data packets omit identity. The lease lasts 45 seconds, so the client needs to keep heartbeating before that lease expires.

## Data-Packet Topic: `ros.subscriptions.status`

`ros.subscriptions.status` is a control-plane status message on a LiveKit data-packet topic. It reports the result of a non-empty heartbeat.

### Example Response (informative)

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

### Envelope Requirements

- `v` MUST be the protocol version and is currently `2`
- `type` MUST always be `ros.subscriptions.status`
- `session_id` MUST be included only when the heartbeat carried a non-blank `session_id`
- `lease_expires_in_ms` MUST be included on every non-empty status packet
- `lease_expires_in_ms` MUST be treated as approximate because the bridge computes it at serialization time
- the bridge MUST publish no status packet when the heartbeat produced an empty `subscriptions` array

### Per-Subscription Status Objects

Active statuses MUST include:

- `kind`: `topic` or `other_video`
- `name`
- `status`: `active`

Error statuses MUST include:

- `kind`
- `name`
- `status`: `error`
- `error.reason`
- `error.message`

Current `error.reason` values:

- `forbidden`: the subscribe policy denies a topic
- `unavailable`: the bridge could not start or keep running a required runtime dependency, usually a video stream pipeline
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
    "track_name": "ros.data.battery_state",
    "content_type": "application/x-ros-cdr",
    "interval_ms": 100
  }
}
```

Requirements:

- `track_name` MUST be deterministic: the bridge prefixes `ros.data` and replaces `/` with `.`
- `delivery.interval_ms` MUST always be present for data deliveries, including `0`
- bytes sent on the data track MUST be raw serialized CDR bytes, not nested JSON

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
    "track_name": "ros.video.other.front_camera"
  }
}
```

Requirements:

- `other_video` targets MUST always use video delivery
- ROS topics MUST use video delivery only when their resolved type is `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage`
- video `track_name` values MUST be deterministic and stable for the target name
- `other_video` track names MUST percent-encode any byte outside RFC 3986 unreserved characters

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

### Requirements

- `service` and `request` MUST be present
- `service` MUST normalize to a non-empty ROS service name
- `request.payload_base64` MUST decode to a non-empty byte vector
- `interface_type` is optional; if omitted or blank, the bridge MUST require exactly one graph-advertised service type
- `timeout_ms`, when present, MUST be an integer
- values `<= 0` MUST NOT disable timeouts; they MUST fall back to the bridge default of `2000` ms
- the bridge MUST check access policy after request parsing and before the ROS request is sent
- each requester identity MUST be limited to at most `4` in-flight service calls
- some failures MAY happen after request acceptance, including timeout, requester disconnect, session reset, or shutdown

### Notes (informative)

Clients that omit `interface_type` should do so only when they are prepared for ambiguity to fail the call.

## RPC: `ros2.interface.show`

`ros2.interface.show` returns interface definitions needed to encode or decode ROS payloads.

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

### Requirements

- `interface_types` MUST be present and MUST be a non-empty array
- every array entry MUST trim to a non-empty string
- the bridge MUST return the requested definition first, then any transitive message dependencies
- duplicates MUST be removed while preserving first discovery order
- `definition` MUST be the raw `.msg` or `.srv` file content from the package share directory

### Notes (informative)

This method exists so a client can obtain message and service definitions before it tries to serialize or deserialize CDR payloads.
Despite the singular command-style name, the JSON request remains batch-oriented so clients can fetch multiple interface definitions in one round-trip.

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

### Requirements

- `query` is optional
- missing, `null`, and blank `query` values MUST be treated as absent
- `limit` is optional, but when present it MUST be a positive integer
- filtering MUST happen after the ROS graph query and after access-policy checks
- `query` MUST match substrings in either the resource name or the interface type
- resources with zero or multiple interface types MUST be skipped instead of being returned ambiguously

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

### Requirements

- `query` is optional
- missing, `null`, and blank `query` values MUST be treated as absent
- `limit` is optional, but when present it MUST be a positive integer
- filtering MUST happen after the ROS graph query and after access-policy checks
- `query` MUST match substrings in either the resource name or the interface type
- resources with zero or multiple interface types MUST be skipped instead of being returned ambiguously

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
- when the last requester lease disappears, the shared data track or video stream MUST be torn down

### Example (informative)

If two requesters subscribe to the same normalized non-video ROS topic, they share one ROS subscription and one data track. The bridge tracks separate leases, but not separate data-track backends for that topic.

## Reconnect and Lease Semantics

Requirements:

- each heartbeat-renewed subscription lease lasts 45 seconds from the renewing heartbeat
- omitting a target from a later heartbeat MUST NOT cancel its existing lease immediately
- the bridge MUST allow leases to expire naturally when they are not renewed
- if a requester reconnects and still owns a live data subscription, the bridge MUST republish the data track after a heartbeat confirms the requester
- a well-formed subscription heartbeat MAY therefore trigger one data-track republish after a page refresh or participant reconnect

### Example (informative)

If a browser refreshes but sends another valid heartbeat before the old lease expires, the bridge can re-announce the same data-track delivery without recreating the subscription from scratch.

## Error Model

Data-packet topics and RPCs fail differently:

- malformed `ros2.topic.pub` packets, unsupported data-packet topics, and anonymous publish requests are dropped after logging
- malformed subscription heartbeats are dropped after logging
- well-formed subscription heartbeats report per-target failures through `ros.subscriptions.status`
- RPC failures surface through LiveKit RPC errors

Stable RPC error codes:

| Code | Meaning | Typical causes |
| --- | --- | --- |
| `2400` | invalid request | bad JSON, wrong field types, empty canonical names, payload decode failures, invalid bounds |
| `2401` | unauthorized | missing `caller_identity` |
| `2403` | forbidden | access policy denies the requested service or topic |
| `2500` | internal | ROS graph failures, client creation failures, runtime exceptions, late service-call failures |

The bridge MUST map `std::invalid_argument` and `std::out_of_range` to `2400`. Everything else MUST become `2500` unless the code already raised a more specific `RpcHandlerError`.

## Informative Examples

### Service Call Flow (informative)

One common request-response path looks like this:

1. Call `ros2.service.list` to discover an allowed service.
2. Call `ros2.interface.show` for the service type.
3. Serialize the request payload as ROS CDR.
4. Call `ros2.service.call`.
5. Decode the returned ROS CDR response payload.

### Topic Subscription Flow (informative)

One common data-subscription path looks like this:

1. Send `ros.subscriptions.heartbeat` with a `topic` subscription request.
2. Read `ros.subscriptions.status`.
3. If the status is `active` and `delivery.kind` is `data`, subscribe to the announced LiveKit data track.
4. Decode incoming bytes on that track as raw ROS CDR for the reported `interface_type`.

### Other Video Flow (informative)

One common non-ROS video path looks like this:

1. Send `ros.subscriptions.heartbeat` with `kind: "other_video"` and the configured source id as `name`.
2. Read `ros.subscriptions.status`.
3. If the status is `active` and `delivery.kind` is `video`, subscribe to the announced LiveKit video publication.

## Informative Typical Client Flow

Most integrations follow this order:

1. Join the same LiveKit room as the bridge.
2. Call `ros2.topic.list` and `ros2.service.list` to discover only the resources your policy allows.
3. Call `ros2.interface.show` for the message and service types your client needs to encode or decode.
4. Use `ros2.service.call` for request-response operations.
5. Send `ros2.topic.pub` packets for small allowed topic writes.
6. Send `ros.subscriptions.heartbeat` on a regular cadence to request topic or video subscriptions.
7. Read `ros.subscriptions.status` to learn whether each requested subscription is active, forbidden, unavailable, or not found.
8. Subscribe to the announced LiveKit data track or video publication.

For a first integration, start with one service-call path or one topic-subscription path. Once that works, add more interface types, video, and broader policy rules.

## ROS 2 Command Mapping (informative)

This bridge mirrors the ROS 2 CLI at the entrypoint-name level, not at the payload-format level. Request-response work uses LiveKit RPCs, one-shot topic writes use LiveKit data-packet topics, and streaming deliveries arrive on LiveKit data or video tracks.

| Common ROS 2 command / mental model | Bridge protocol surface(s) | What changes in LiveKit |
| --- | --- | --- |
| `ros2 topic list` | RPC `ros2.topic.list` | Returns the allowed ROS topics the caller may use, including each topic's interface type. |
| `ros2 service list` | RPC `ros2.service.list` | Returns the allowed ROS services the caller may use, including each service's interface type. |
| `ros2 interface show <type>` | RPC `ros2.interface.show` | Clients fetch raw ROS interface definitions here before they encode or decode ROS CDR payloads. The request body still supports batching via `interface_types`. |
| `ros2 service call /service Type ...` | RPC `ros2.service.call` | Request and response bodies use the shared JSON CDR envelope with base64 payload bytes, not ROS CLI text formatting. |
| `ros2 topic pub /topic Type ...` | Data-packet topic `ros2.topic.pub` | This is a best-effort one-message write path for small allowed publishes. The bridge sends no acknowledgement packet. |
| `ros2 topic echo /topic` for a non-video topic | Data-packet topic `ros.subscriptions.heartbeat` -> data-packet topic `ros.subscriptions.status` -> LiveKit data track | The heartbeat requests the subscription, the status packet reports whether it became active and names the track, and the track carries raw ROS CDR bytes. |
| Subscribing to an image topic such as `/camera/image_raw` | Data-packet topic `ros.subscriptions.heartbeat` -> data-packet topic `ros.subscriptions.status` -> LiveKit video track | ROS image topics may resolve to video delivery instead of a data track when the bridge treats the topic as video. |
| Subscribing to a configured non-ROS video source | Data-packet topic `ros.subscriptions.heartbeat` with `kind: "other_video"` -> data-packet topic `ros.subscriptions.status` -> LiveKit video track | There is no direct ROS CLI equivalent for `other_video`; it addresses a configured bridge-owned video source. |
| `ros2 action *` | No protocol equivalent | ROS actions are not supported by this package today. |
| `ros2 param *` | No protocol equivalent | ROS parameter get and set are not supported by this package today. |

The easy-to-miss part is that `ros.subscriptions.heartbeat` and `ros.subscriptions.status` are bridge-specific control-plane messages, not standard ROS messages. For non-video topics, the announced LiveKit data track carries raw ROS CDR bytes rather than JSON, and clients should use the `track_name` reported in `ros.subscriptions.status` instead of treating track naming as a separate discovery flow.
