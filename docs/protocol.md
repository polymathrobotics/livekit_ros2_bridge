# Client protocol (LiveKit ↔ ROS 2)

This document defines the client-facing contract implemented by `livekit_ros2_bridge`.

Everything is JSON.

## Names

- Data topics: `ros.topic.publish` (LiveKit → ROS), `ros.topic.messages` (ROS → LiveKit)
- RPC methods: `ros.topic.subscribe`, `ros.topic.unsubscribe`, `ros.service.call`

ROS topic/service names are normalized by the bridge (leading `/`, duplicate slashes collapsed, trailing `/` removed).

## Data topics

### Publish into ROS: `ros.topic.publish`

Send a JSON object on `ros.topic.publish`:

```json
{
  "topic": "/user/cmd_vel",
  "type": "geometry_msgs/msg/Twist",
  "msg": {
    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
  }
}
```

Notes:

- `type` must be a fully-qualified ROS interface type string.
- `msg` must match the fields for `type`. Unknown fields are ignored.
- Policy is enforced before any ROS publishers are created.

### Stream ROS messages out: `ros.topic.messages`

The bridge publishes ROS messages on `ros.topic.messages` using this envelope:

```json
{
  "v": 1,
  "type": "ros.message",
  "id": "msg-1",
  "body": {
    "topic": "/example/topic",
    "type": "std_msgs/msg/String",
    "msg": {"data": "hello"},
    "content_type": "application/json"
  }
}
```

Notes:

- `v` is the protocol version (currently `1`).
- `id` increases per bridge instance.
- Streams are enabled/disabled via `ros.topic.subscribe` / `ros.topic.unsubscribe`.
- Streamed data is delivered only to subscribed identities (targeted using LiveKit `destination_identities`).

## RPC

### `ros.topic.subscribe`

Request:

```json
{
  "topic": "/example/topic",
  "preferred_interval_ms": 100
}
```

Response:

```json
{
  "ok": true,
  "subscription": {
    "topic": "/example/topic",
    "type": "std_msgs/msg/String",
    "reliability": "reliable",
    "depth": 10
  },
  "status": {
    "applied_interval_ms": 100,
    "requester_count": 1
  }
}
```

Notes:

- `preferred_interval_ms` is optional. It must be non-negative; `0` (or omission) disables rate limiting.
- If multiple requesters subscribe to the same topic, the applied interval is the minimum across requesters.
- The bridge uses LiveKit `caller_identity` as the requester identity for subscription tracking.
- The bridge uses the requester identity as the LiveKit `destination_identity` for delivering `ros.topic.messages`.
- Subscription QoS is currently fixed to RELIABLE with depth 10.

### `ros.topic.unsubscribe`

Request:

```json
{
  "topic": "/example/topic"
}
```

Response matches `ros.topic.subscribe`.

### `ros.service.call`

Request:

```json
{
  "service": "/example/service",
  "type": "example_interfaces/srv/AddTwoInts",
  "request": {"a": 1, "b": 2},
  "timeout_ms": 1000
}
```

Response:

```json
{
  "ok": true,
  "service": {"name": "/example/service", "type": "example_interfaces/srv/AddTwoInts"},
  "response": {"sum": 3},
  "elapsed_ms": 12
}
```

Notes:

- `type` is optional; if omitted, the bridge resolves the service type from the ROS graph.
- `timeout_ms` falls back to the configured default if omitted or non-positive.
- Policy is enforced before calling the service.

## Errors

RPC failures are returned as LiveKit RPC errors with a numeric error code and message.

The bridge uses HTTP-like status codes (in the `2xxx` range) so clients can bucket failures
without needing a bespoke taxonomy:

- `2400` `INVALID_REQUEST`: request failed validation / parsing, or was otherwise invalid
- `2401` `UNAUTHORIZED`: LiveKit `caller_identity` unavailable (required for RPC)
- `2403` `FORBIDDEN`: blocked by access rules or a limit
- `2500` `INTERNAL`: service unavailable / timeout / unexpected internal failure

Treat the numeric code as the stable contract; surface the message for debugging.

## Serialization

ROS messages and service responses are converted to JSON:

- Non-finite floats (`NaN`, `+Inf`, `-Inf`) become `null`
- `bytes`/`bytearray` become lists of integers
- Objects with a `tolist()` method may be converted via `tolist()` recursively

This protocol does not define a binary payload format; large or high-rate messages are not a good fit for data channels.
