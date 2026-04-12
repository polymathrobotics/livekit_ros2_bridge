# Protocol reference

This page covers the stable LiveKit-facing contract for RPC methods and the non-subscription control path. Subscription heartbeat and status behavior lives in [subscriptions.md](./subscriptions.md), because the wire format and runtime lease model are tightly coupled.

## Surface summary

| Surface | Name | Direction | Purpose |
| --- | --- | --- | --- |
| Control topic | `ros.topics.publish` | caller -> bridge | Best-effort ROS topic publication |
| Control topic | `ros.subscriptions.heartbeat` | caller -> bridge | Request and renew subscriptions |
| Control topic | `ros.subscriptions.status` | bridge -> caller | Per-stream subscription status |
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
- normalized ROS names are absolute-style names after `normalizeRosResourceName()`
- if normalization produces an empty name, the request is invalid
- anonymous RPC calls are rejected up front
- anonymous `ros.topics.publish` packets are dropped
- anonymous subscription heartbeats are accepted only through the `session_id` fallback described in [subscriptions.md](./subscriptions.md)

The current subscription status envelope uses protocol version `2`.

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

## Subscription control topics

`ros.subscriptions.heartbeat` and `ros.subscriptions.status` are documented in [subscriptions.md](./subscriptions.md). That page covers:

- request and response payloads
- `session_id` fallback behavior
- lease timing
- per-stream error reasons
- CDR replay after reconnect

## Error handling

Control topics and RPCs fail differently:

- control-topic failures are local-only; malformed packets, unsupported control topics, and anonymous publish commands are dropped after logging
- RPC failures surface through LiveKit RPC errors

Stable RPC error codes:

| Code | Meaning | Typical causes |
| --- | --- | --- |
| `2400` | invalid request | bad JSON, wrong field types, empty canonical names, payload decode failures, invalid bounds |
| `2401` | unauthorized | missing `caller_identity` |
| `2403` | forbidden | access policy denies the requested service or topic |
| `2500` | internal | ROS graph failures, client creation failures, runtime exceptions, late service-call failures |

The bridge maps `std::invalid_argument` and `std::out_of_range` to `2400`. Everything else becomes `2500` unless the code already raised a more specific `RpcHandlerError`.
