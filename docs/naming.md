# Naming Guide

Use these names consistently in `livekit_ros2_bridge`.

This guide is for internal code, docs, tests, comments, and logs. It does not define the wire
protocol. For external payloads and field names, see [protocol.md](./protocol.md) and
[subscriptions.md](./subscriptions.md).

## Cross-Cutting

- `config`: Startup configuration loaded from parameters. It does not change while the bridge is running.
- `spec`: Resolved runtime input built from a `config` plus a specific request or lookup.
- `registry`: Shared manager that owns deduplicated resources.
- `instance`: One live shared runtime that connects an input to an output.
- `publisher`: Component responsible for pushing data to a destination.
- `source`: Input-side runtime that produces frames or messages.
- `router`: Component that sends inbound LiveKit traffic to the right handler.
- `lease`: Time-limited ownership or identity binding.

## Data (ROS -> LiveKit)

- `instance`: One live logical data stream per normalized ROS topic, such as `DataStreamInstance`.
- `subscription`: ROS `rclcpp::GenericSubscription` input side.
- `publisher`: One LiveKit data track owner, such as `DataTrackPublisher`.
- `payload`: Message envelope and serialization helpers.
- `cdr`: Payload encoding only. Never the transport.
- `track`: LiveKit transport object only, such as `livekit::LocalDataTrack`.
- `stream`: The full runtime from ROS subscription to LiveKit delivery.

## Video (ROS / GStreamer -> LiveKit)

- `config`: Declared video configuration, such as `RosVideoTopicRule` or `ConfiguredVideoStreamSource`.
- `spec`: Resolved video input and publish settings, such as `VideoStreamSpec`.
- `instance`: One live shared video runtime per `stream_key`, such as `VideoStreamInstance`.
- `source`: Frame-producing input side, such as `VideoFrameSource`.
- `publisher`: One LiveKit video track owner, such as `VideoTrackPublisher`.
- `track`: LiveKit transport object only, such as `livekit::LocalVideoTrack` or `PublishedVideoTrack`.
- `stream`: The full logical video delivery runtime.

## RPC & Service Calls

- `caller`: Component that executes the ROS service request, such as `RosServiceCaller`.
- `invocation`: Inbound LiveKit RPC execution context, such as `RpcInvocation`.
- `request` / `response`: Parsed payloads for one RPC method, such as `ServiceCallRequest` and `ServiceCallResponse`.
- `inflight`: Active unresolved requests that count against a quota.

## Control Messages

- `command`: Fire-and-forget control request.
- `heartbeat`: Lease renewal message.
- `demand`: One requested subscription carried inside a heartbeat.
- `status`: Reported control result.
- `update`: Fire-and-forget state change without RPC response.

## Access Control & Lookup

- `policy`: Top-level authorization manager, such as `AccessPolicy`.
- `operation`: Action being authorized, such as `AccessOperation::Publish`, `Subscribe`, or `CallService`.
- `rule`: One allow or deny rule inside a policy.
- `resource`: Real ROS graph entity used for lookup or access checks.
- `target`: Caller-selected identifier. A target may resolve to a ROS resource, but it can also be a configured source.

## Connections, Sessions, and Leases

- `connection`: The bridge's LiveKit room transport lifetime.
- `session`: A requester or participant lifetime. Do not use it for the bridge's room transport.
- `lease`: Time-limited ownership or identity binding. Use it for shared stream ownership and related fallback bindings.

## Concurrency & Lifecycles

- `queue`: Handoff across threads so work runs on the correct ROS executor thread, such as `RosExecutorQueue`.
- `drain`: Run or empty queued work on the owning thread.
- `quiesce`: Stop new work from entering a callback or scope while in-flight work finishes.
- `ensureRunning`: Start or initialize a runtime object if it is not already running.

## Boundaries & Practical Rules

- `data` vs `video`: Describe the media type on the LiveKit side.
- `config` vs `spec`: Use `config` for declared parameters and `spec` for resolved runtime data.
- `cdr` vs `track` vs `stream`: `cdr` is encoding, `track` is LiveKit transport, and `stream` is the bridge's end-to-end runtime.
- `source`: Use `source` only when there is a real input-side runtime behind the name.
- `request` vs control messages: Use `request` and `response` for RPC-style calls. Use `command`, `heartbeat`, `demand`, `status`, and `update` for control-path concepts.
- `resource` vs `target`: `resource` is a real ROS graph entity. `target` may also be a configured source.
- `connection` vs `session` vs `lease`: `connection` is bridge transport state, `session` is requester or participant lifetime, and `lease` is time-limited ownership.
