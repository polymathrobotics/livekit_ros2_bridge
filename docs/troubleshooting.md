# Troubleshooting

This page covers the common failures people hit during setup, connection, subscriptions, video, and testing. Start with the symptom, then follow the linked reference page if you need the deeper contract or runtime details.

## Start with the symptom

| Symptom | First events to check | Typical causes |
| --- | --- | --- |
| Node exits during startup | `runtime_config_load_failed`, `node_startup_failed`, `runtime_startup_failed` | missing `livekit.url`, missing `livekit.room`, missing token, invalid video config |
| Room never stabilizes | `room_token_load_failed`, `room_connect_failed`, `room_reconnect_backoff` | bad or expired token, empty token, unreachable LiveKit URL |
| Heartbeat sent but no usable status returns | `control_packet_rejected`, `heartbeat_dropped`, `heartbeat_session_conflict`, `subscription_status_publish_failed` | malformed JSON, anonymous heartbeat without a valid `session_id`, session ownership mismatch |
| Service call rejected or times out | `rpc_request_rejected`, `service_call_failed`, `service_calls_settled` | missing caller identity, access denied, bad payload, request build failure, timeout, requester disconnect |
| Data-track topic subscription appears but data does not flow | `subscription_renew_failed`, `subscription_qos_resolved`, `data_track_pending`, `data_track_published`, `data_track_publish_failed`, `data_track_delivery_failed`, `data_track_delivery_dropped` | ambiguous topic type, QoS mismatch, track publish failure, LiveKit queue backpressure |
| Video source never appears | `subscription_renew_failed`, `subscription_qos_resolved`, `video_stream_subscription_started`, `video_stream_pipeline_starting`, `video_stream_push_failed`, `video_stream_pipeline_failed` | QoS mismatch, invalid video source or transform config, unsupported image encoding, unhealthy source pipeline |

## Startup and connection issues

### The node exits before it joins the room

Look for:

- `event=runtime_config_load_failed`
- `event=node_startup_failed`
- `event=runtime_startup_failed`

Common causes:

- missing `livekit.url` or `livekit.room`
- missing `livekit.token`
- invalid `video.topic_rules.*` or `video.configured_sources.*` configuration

Start with [runtime-configuration.md](./runtime-configuration.md).

### The bridge starts but never connects

Look for:

- `event=room_token_load_failed`
- `event=room_connect_failed`
- `event=room_connected`

Common causes:

- unreachable LiveKit URL
- bad room name
- bad or expired token

### The bridge keeps reconnecting

Look for:

- `event=room_reconnect_requested`
- `event=room_reconnect_backoff`
- `event=room_session_reset`

Common causes:

- transport loss
- expired startup token
- the room disconnected and the reconnect loop is backing off

## RPC and access issues

### An RPC returns `2401`, `2403`, `2400`, or `2500`

| Code | Meaning | Common causes |
| --- | --- | --- |
| `2401` | unauthorized | missing `caller_identity` |
| `2403` | forbidden | access rules deny the requested service or topic |
| `2400` | invalid request | bad JSON, wrong field types, empty canonical names, invalid base64, invalid limits or timeouts |
| `2500` | internal | ROS graph failures, client creation failures, request build failures, timeout, disconnect, or shutdown after a service call started |

For the exact RPC contract, read [protocol.md](./protocol.md). For rule matching, read [access-control.md](./access-control.md).

### A topic or service is missing from a list response

`ros.topics.list` and `ros.services.list` both filter results after they query the ROS graph.

Check these first:

- the relevant allow and deny rules
- whether the resource has exactly one interface type
- whether your `query` filter matches the resource name or interface type

## Subscription issues

### No status packet comes back

Check these first:

- an empty `subscriptions` array produces no status packet
- anonymous heartbeats without a known `session_id` are dropped
- malformed heartbeats are rejected before processing

Useful log events:

- `event=heartbeat_dropped`
- `event=control_packet_rejected`
- `event=subscription_status_publish_failed`

### A stream entry returns `forbidden`, `not_found`, or `unavailable`

| `error.reason` | What it usually means |
| --- | --- |
| `forbidden` | the subscribe policy denied a topic |
| `not_found` | the topic or configured source could not be resolved, or the topic type was ambiguous |
| `unavailable` | a required runtime dependency could not start or keep running, usually a video stream pipeline |

Remember:

- topic subscriptions use `access.rules.subscribe.*`
- `configured_source` targets do not use those rules; they depend on which `video_configured_source_ids` and `video.configured_sources.*` entries exist

For the heartbeat and status contract, read [subscriptions.md](./subscriptions.md).

### A browser refresh breaks data tracks until the next heartbeat

That is expected.

The bridge can keep the lease state alive across a refresh, but data-track publications belong to the previous participant session. After the next successful heartbeat, the bridge may unpublish and republish the same deterministic `track_name` so the new participant session sees it again.

Useful log events:

- `event=data_track_republish`
- `event=heartbeat_session_fallback`

## Video issues

### Video never appears

Check these first:

- the relevant `video.topic_rules.*` or `video.configured_sources.*` entry exists and matches what the client requested
- configured-source ids are trimmed only; slash and colon variants stay distinct
- the bridge logged `subscription_qos_resolved` for that ROS topic
- the resolved `reliability` and `durability` match the publisher

Common QoS example:

- `ros_gz_bridge` camera topics often publish `RELIABLE`
- if the bridge subscribes before publisher QoS is visible, add `subscription.qos_overrides.*` for that topic pattern

Example:

- `video.configured_sources.front_rtsp.source: ...` is requested as `configured_source: "front_rtsp"`

Useful log events:

- `event=subscription_qos_resolved`
- `event=video_stream_subscription_started`
- `event=video_stream_input_received`
- `event=video_stream_push_failed`
- `event=video_stream_pipeline_failed`

### A ROS topic becomes a data track instead of video

Only two ROS topic types go down the video path:

- `sensor_msgs/msg/Image`
- `sensor_msgs/msg/CompressedImage`

All other ROS topic types stay on the data-track path carrying ROS CDR payloads.

### A video subscription returns `unavailable`

Common causes:

- the configured-source fragment could not be composed into a valid runtime pipeline
- the ROS image encoding or compressed-image format is unsupported
- the source pipeline reached EOS or an error state before it could produce frames
- a LiveKit video track publish failed

For video rule resolution and runtime behavior, read [video-sources.md](./video-sources.md).

## Topic publish and service-call issues

### `ros.topics.publish` seems to do nothing

Remember that `ros.topics.publish` is best-effort and has no acknowledgement packet.

Check these first:

- the caller is not anonymous
- the topic is allowed by `access.rules.publish.*`
- the requested `interface_type` matches the bridge's resolved topic type
- the topic exists with exactly one graph type

Useful log events:

- `event=publish_request_rejected`
- `event=publish_request_failed`

### A service call starts and then fails later

That can happen after the request already left the bridge.

Common causes:

- timeout
- requester disconnect
- session reset
- bridge shutdown
- more than four in-flight service calls for one requester identity

Useful log events:

- `event=service_call_failed`
- `event=service_calls_settled`
- `event=service_response_dropped`

## Test issues

### Tests time out or flake

Replace fixed sleeps first.

- use `waitForTopicType(...)` before tests depend on graph visibility
- use `spinUntil(...)` when executor progress is required
- use `waitUntil(...)` for non-ROS state changes

Useful commands:

```bash
just test
just test --ctest-args -R test_runtime
```

Useful log event:

- `event=wait_for_topic_type_timeout`

For the common harness shape, read [testing.md](./testing.md).

## Reading logs

The bridge's logs are intentionally flat. Most lines start with `event=...` and then add `key=value` pairs.

Two details matter when you read them:

- `count=` means aggregated repeats inside a throttle window, not severity
- throttled warnings without `count=` can still hide repeated failures

When in doubt, find the earliest log line for the affected resource and follow the lifecycle from there.
