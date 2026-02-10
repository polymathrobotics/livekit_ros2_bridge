# Troubleshooting

## Symptom: bridge exits on startup

Likely causes:

- All allowlists are empty
- Missing `livekit.url` or `livekit.room`
- Missing auth (`livekit.token` or `livekit.api_key` + `livekit.api_secret`)

Checks:

- Validate overlay YAML has at least one non-empty allowlist entry
- Confirm required env vars are exported in the shell launching ROS
- Confirm params are keyed under `livekit_bridge.ros__parameters`

Fix:

- Set at least one of `access.static.publish.allow`, `access.static.subscribe.allow`, or `access.static.service.allow`
- Set connection and auth parameters in YAML or via `$(env ...)` inputs

## Symptom: no ROS -> LiveKit data on `ros.topic.messages`

Likely causes:

- `ros.topic.subscribe` was never called (or failed)
- Topic is blocked by access rules
- Topic type is blocked by `access.static.subscribe.type_deny`
- Subscriber identity is no longer present in the room

Checks:

- Verify successful `ros.topic.subscribe` response from the client
- Verify topic/type access config in your overlay
- Verify the participant identity is still connected

Fix:

- Use an allowed topic and valid request payload
- Adjust allowlist/denylist/type-denylist access rules as needed

## Symptom: `ros.topic.subscribe` returns invalid request or type errors

Likely causes:

- Topic is not present in ROS graph yet (no publishers)
- Topic has multiple ROS types
- Requested topic is blocked by access rules

Checks:

- Inspect topic graph and types:

  ```bash
  ros2 topic list -t
  ```

- Confirm the topic resolves to one type
- Confirm the topic name matches access patterns

Fix:

- Start a publisher for the topic before subscribing
- Ensure only one type is used for the topic
- Update allowlist/denylist patterns

## Symptom: LiveKit -> ROS publish requests fail

Likely causes:

- Topic blocked by publish access rules
- Payload schema mismatch
- Payload `type` does not resolve, or does not match ROS graph type

Checks:

- Confirm topic access rules allow publishing
- Validate payload has `topic`, `type`, and `msg`
- Confirm ROS graph reports the same type for that topic

Fix:

- Correct topic/type/payload fields
- Align publisher type with ROS graph type

Related docs:

- [configuration.md](configuration.md)
- [protocol.md](protocol.md)
