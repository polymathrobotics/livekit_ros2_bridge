# Client Interface and Protocol Specification

This document is the authoritative specification for the client-facing interface and protocol of `livekit_ros2_bridge`, with explanatory material where useful.

## Typical client flow

Most integrations follow this order:

1. Join the same LiveKit room as the bridge.
2. Call `ros.topics.list` and `ros.services.list` to discover only the resources your policy allows.
3. Call `ros.interfaces.get` for the message and service types your client needs to encode or decode.
4. Use `ros.services.call` for request-response operations.
5. Send `ros.topics.publish` packets for small allowed topic writes.
6. Send `ros.subscriptions.heartbeat` on a regular cadence to request topic or video subscriptions.
7. Read `ros.subscriptions.status` to learn whether each requested subscription is active, forbidden, unavailable, or not found.
8. Subscribe to the announced LiveKit data track or video publication.

For a first integration, start with one service call or one topic subscription path. Once that works, add more interface types, video, and broader policy rules.

## Surface summary

| Surface | Name | Direction | Purpose |
| --- | --- | --- | --- |
| Control topic | `ros.topics.publish` | caller -> bridge | Best-effort ROS topic publication |
| Control topic | `ros.subscriptions.heartbeat` | caller -> bridge | Request and renew subscriptions |
| Control topic | `ros.subscriptions.status` | bridge -> caller | Per-subscription status |
| RPC | `ros.services.call` | caller <-> bridge | Call an authorized ROS service |
| RPC | `ros.interfaces.get` | caller <-> bridge | Fetch interface definitions |
| RPC | `ros.services.list` | caller <-> bridge | List authorized ROS services |
| RPC | `ros.topics.list` | caller <-> bridge | List authorized ROS topics |

## Shared wire rules

ROS binary payloads use the same JSON envelope everywhere:

```json
{
  "content_type": "application/x-ros-cdr",
  "payload_base64": "AAECAw=="
}
```

Important rules:

- `payload_base64` must be padded standard base64
- wrong `content_type`, wrong JSON types, and invalid base64 are rejected
- parsers trim string fields before validation
- blank optional strings are usually treated as missing
- unknown JSON fields are ignored unless a specific parser rejects them
- ROS names are normalized to absolute-style names before validation and policy checks
- if normalization produces an empty name, the request is invalid
- anonymous RPC calls are rejected up front
- anonymous `ros.topics.publish` packets are dropped
- anonymous `ros.subscriptions.heartbeat` packets are accepted only through the `session_id` fallback described below

## Control topic: `ros.topics.publish`

Request body:

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

Behavior:

- `topic` is required, must be a string, and must normalize to a non-empty ROS topic name
- `interface_type` is required, trimmed, and must stay non-empty
- `message` is required and must decode to a non-empty CDR payload
- the bridge checks publish authorization against the normalized topic name
- the requested `interface_type` must match the bridge's resolved topic type exactly
- once the bridge has cached a publisher for a topic, later requests are checked against that cached type instead of re-reading the ROS graph
- this path is best-effort and has no acknowledgement packet; malformed, forbidden, or late requests are logged and dropped

## Control topic: `ros.subscriptions.heartbeat`

Request body:

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

Behavior:

- `subscriptions` is required and must be an array
- each subscription entry must be an object with string `kind` and `name` fields
- `kind` must be `topic` or `configured_source`
- `topic` names normalize as ROS resource names
- `configured_source` names address configured entries from `video.other.<id>`
- `delivery_preferences` is optional and must be an object when present
- `delivery_preferences.interval_ms` is optional and must be an integer when present
- `interval_ms: 0` means no preference
- repeating the same target yields one effective request; the smallest non-zero `interval_ms` wins
- negative `interval_ms` values clamp to `0` when the lease is applied
- `session_id` is optional; missing, `null`, and blank values are treated as absent
- each heartbeat renews the listed subscriptions for 45 seconds
- omitting a previously requested target leaves its existing lease active until expiry
- a heartbeat with an empty `subscriptions` array renews nothing and publishes no status packet

### Requester identity and `session_id`

The bridge prefers the LiveKit packet's `requester_identity`. `session_id` exists only to keep heartbeats working when LiveKit omits that identity from user-data packets.

Behavior:

- heartbeats with a non-empty `requester_identity` are accepted normally
- if that heartbeat also includes `session_id`, the bridge binds the `session_id` to that requester for 45 seconds
- a later heartbeat with an empty `requester_identity` is accepted only if it includes a known, unexpired `session_id`
- a `session_id` cannot be rebound to a different requester until the existing lease expires
- anonymous heartbeats without a known `session_id` are dropped

## Control topic: `ros.subscriptions.status`

Response body:

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
- `session_id` is included only when the heartbeat carried a non-blank `session_id`
- `lease_expires_in_ms` is included on every non-empty status packet and is approximate because it is computed at serialization time
- the bridge publishes no status packet when the heartbeat produces an empty `subscriptions` array

### Per-subscription status objects

Active statuses always include:

- `kind`: `topic` or `configured_source`
- `name`
- `status`: `active`

Error statuses always include:

- `kind`
- `name`
- `status`: `error`
- `error.reason`
- `error.message`

Current `error.reason` values:

- `forbidden`: the subscribe policy denies a topic
- `unavailable`: the bridge could not start or keep running a required runtime dependency, usually a video stream pipeline
- `not_found`: lookup or subscription creation failed for another reason

### Active topic subscriptions on a data track

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

### Active video subscription entries

Video deliveries use deterministic track names:

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

- `configured_source` targets always use video delivery
- ROS topics use video delivery only when their resolved type is `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage`
- video `track_name` is deterministic and stable for the target name
- configured-source video track names percent-encode any byte outside RFC 3986 unreserved characters

## Delivery and sharing model

| Requested target | Resolved from | Delivery | Shared resource model |
| --- | --- | --- | --- |
| Non-video ROS topic | normalized topic name and unique graph type | data | one ROS subscription and one data track per normalized topic |
| ROS video topic | normalized topic name, unique graph type, and matching video topic entry | video | one in-process video stream per resolved `stream_key` |
| Configured source | matching `video.other.*` entry | video | one in-process video stream per resolved `stream_key` |

Important behavior:

- topic subscriptions use `access.rules.subscribe.*`
- `configured_source` targets do not use subscribe rules; they are controlled by which `video_other_ids` and `video.other.*` entries exist
- the bridge never guesses when topic type resolution is ambiguous
- when the last requester lease disappears, the shared data track or video stream is torn down

## Reconnect behavior

- if a requester reconnects and still owns a live data subscription, the bridge republishes the data track after a heartbeat confirms the requester
- well-formed subscription heartbeats can therefore trigger one data-track republish after a page refresh or participant reconnect

## RPC: `ros.services.call`

Request body:

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

Successful response body:

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

Behavior:

- `service` and `request` are required
- `request.payload_base64` must decode to a non-empty byte vector
- `interface_type` is optional; if omitted or blank, the bridge requires exactly one graph-advertised service type
- `timeout_ms` must be an integer
- values `<= 0` do not disable timeouts; they fall back to the bridge default of `2000` ms
- access policy is checked after request parsing and before the ROS request is sent
- each requester identity can have at most `4` in-flight service calls
- some failures happen later than request acceptance, such as timeout, requester disconnect, session reset, or shutdown

## RPC: `ros.interfaces.get`

Request body:

```json
{
  "interface_types": [
    "sensor_msgs/msg/Image",
    "std_srvs/srv/Trigger"
  ]
}
```

Successful response body:

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

Behavior:

- `interface_types` is required and must be a non-empty array
- every array entry must trim to a non-empty string
- the bridge returns the requested definition first, then any transitive message dependencies
- duplicates are removed while preserving first discovery order
- `definition` is the raw `.msg` or `.srv` file content from the package share directory

## RPC: `ros.services.list` and `ros.topics.list`

Shared request body:

```json
{
  "query": "image",
  "limit": 10
}
```

Successful response bodies:

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

Behavior:

- `query` is optional
- missing, `null`, and blank `query` values are treated as absent
- `limit` is optional, but when present it must be a positive integer
- filtering happens after the ROS graph query and after access-policy checks
- `query` matches substrings in either the resource name or the interface type
- resources with zero or multiple interface types are skipped instead of being returned ambiguously

## Error handling

Control topics and RPCs fail differently:

- malformed control packets, unsupported control topics, and anonymous publish requests are dropped after logging
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

The bridge maps `std::invalid_argument` and `std::out_of_range` to `2400`. Everything else becomes `2500` unless the code already raised a more specific `RpcHandlerError`.
