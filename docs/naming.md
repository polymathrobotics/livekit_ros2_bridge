# Naming guide

This guide defines the internal implementation vocabulary for `livekit_ros2_bridge`.

It applies to:

- C++ type and function names
- internal documentation
- code comments
- test names and helpers
- log/event naming where a stable implementation concept is being described

It does not change the LiveKit-facing wire contract. Control topic names, RPC method names, JSON field names, and payload shapes stay as documented in [protocol.md](./protocol.md) and [subscriptions.md](./subscriptions.md) until they are explicitly revised.

## Scope

- Internal names may evolve independently from wire names.
- When wire and internal terms differ, use the protocol docs for client behavior and this guide for implementation names.
- Prefer the narrowest correct layer term:
  - wire: `command`, `heartbeat`, `status`, `request`, `response`
  - runtime: `config`, `spec`, `registry`, `instance`, `source`, `track`, `stream`, `lease`
  - concurrency: `queue`, `drain`, `quiesce`, `ensureRunning`

## Cross-cutting terms

### `config`

Declared startup configuration loaded from parameters. A `config` is not request-specific and is treated as immutable during runtime.

Examples:

- `RuntimeConfig`
- `VideoStreamConfig`
- `RosVideoTopicRule`
- `ConfiguredVideoStreamSource`

### `spec`

Resolved runtime inputs produced from a `config` plus a specific request or lookup result. A `spec` carries concrete values the runtime can execute.

Examples:

- `VideoStreamSpec`

### `registry`

Central manager and lifecycle owner for shared, deduplicated resources.

Examples:

- `SubscriptionRegistry`
- `VideoStreamRegistry`

### `instance`

One live shared logical runtime tying an input to an output.

Examples:

- `DataStreamInstance`
- `VideoStreamInstance`

### `source`

A real input-side runtime abstraction that produces frames or messages for another component. Do not use `source` for a mere identifier, config entry, or upstream system unless there is an actual producing runtime behind it.

Examples:

- `VideoFrameSource`

### `publisher`

Reserved for ROS -> LiveKit publication ownership. A `publisher` owns or maintains a LiveKit publication and does not mean LiveKit -> ROS ingress.

Examples:

- `DataTrackPublisher`
- `VideoTrackPublisher`

### `writer`

Preferred for LiveKit -> ROS best-effort writes. A `writer` executes ingress commands into ROS and is distinct from a LiveKit publication owner.

Preferred example:

- `RosTopicWriter`

### `router`

Distributes inbound LiveKit traffic to the appropriate internal handler. A `router` decides where work goes; it does not own the downstream runtime state.

Examples:

- `RpcRouter`
- `ControlPacketRouter`

### `lease`

Time-bound ownership of a shared runtime or identity binding tied to a requester or client lifecycle.

Examples:

- requester lease
- stream lease
- session lease

### `track`

LiveKit transport object or deterministic LiveKit transport surface only.

Examples:

- `livekit::LocalDataTrack`
- `PublishedVideoTrack`
- `track_name`

### `stream`

The bridge's end-to-end logical delivery runtime. A `stream` is broader than a LiveKit track and may include ROS subscriptions, GStreamer pipelines, leases, and republish behavior.

Examples:

- `DataStreamInstance`
- `VideoStreamInstance`
- `stream_key`

### `payload`

Message envelope or serialization helper. Use `payload` for bytes and envelope parsing, not for transport or lifecycle concepts.

### `cdr`

Payload encoding only. `cdr` describes serialized ROS bytes and never the transport or delivery surface.

## Directionality

This boundary is important enough to be explicit:

- `Publisher` means ROS -> LiveKit.
- `Writer` means LiveKit -> ROS.
- `Track` means LiveKit transport.
- `Stream` means the bridge runtime spanning source, publication, and lease behavior.

The preferred internal ingress name is `RosTopicWriter`. Existing uses of `RosTopicPublisher` are legacy terminology and should migrate toward `RosTopicWriter` as code is touched.

## Data pipeline

For ROS topic data delivered on a LiveKit data track:

- `instance`: one live logical data stream per normalized ROS topic
- `subscription`: the ROS `rclcpp::GenericSubscription` input side
- `publisher`: one LiveKit data-track owner
- `payload`: message envelope and serialization helpers
- `cdr`: serialized ROS bytes only
- `track`: the LiveKit data transport object or name
- `stream`: the full runtime from ROS subscription through LiveKit delivery

Examples:

- `DataStreamInstance`
- `DataTrackPublisher`

## Video pipeline

For ROS or configured-source video delivered on a LiveKit video track:

- `config`: declared video configuration
- `spec`: resolved video stream input and publish settings
- `instance`: one live shared video runtime per `stream_key`
- `source`: frame-producing input runtime
- `publisher`: one LiveKit video-track owner
- `track`: the LiveKit video transport object or name
- `stream`: the logical video delivery runtime

Examples:

- `RosVideoTopicRule`
- `ConfiguredVideoStreamSource`
- `VideoStreamSpec`
- `VideoFrameSource`
- `VideoTrackPublisher`

## RPC and control paths

### RPC

Reserve `request` and `response` for RPC-style interactions that expect a paired response.

Examples:

- `ServiceCallRequest`
- `ServiceCallResponse`
- `RpcInvocation`
- `RosServiceCaller`
- `inflight` requests for quota-tracked unresolved RPC work

### Control topics

For fire-and-forget control traffic, prefer:

- `Command`
- `Heartbeat`
- `Status`
- `Entry`
- `Update`

Avoid introducing new internal `Request` names for fire-and-forget control payloads unless the wire contract already forces that name.

Examples:

- `TopicPublishCommand`
- `SubscriptionHeartbeat`
- `StreamStatus`

## Access control and lookup

### `policy`

The top-level authorization manager.

Example:

- `AccessPolicy`

### `operation`

The action being authorized.

Example:

- `AccessOperation::Publish`
- `AccessOperation::Subscribe`
- `AccessOperation::CallService`

### `rule`

One allow or deny matching rule inside a policy.

### `resource`

A real ROS graph entity normalized for access checks and lookup helpers.

Examples:

- ROS topic names
- ROS service names

### `target`

What the caller asked for on a control or subscription surface. A target may resolve to a ROS resource, but it does not have to.

Examples:

- a topic subscription target
- a `configured_source` target

Rule:

- use `resource` only when the value must be a ROS graph entity
- use `target` when the value may be either a ROS topic or a configured source

For mixed topic and configured-source runtime state, prefer names like `target_name`, `target_kind`, or `target_key` instead of a generic `resource` field.

## Sessions and leases

The codebase currently has two different ideas hiding behind `session`:

- the bridge's LiveKit room lifecycle owner
- the remote client or browser lifecycle used by heartbeat `session_id` fallback

Guidance:

- use `RoomSession` for the LiveKit room connection lifecycle
- qualify requester-side session concepts as `client_session`, `tab_session`, or `requester_session` in new internal names and comments
- avoid introducing new bare `session` names for requester/browser lifecycle state
- keep `lease` separate from connection lifecycle; a lease is time-bound ownership, not a transport connection

This keeps `session_id` fallback understandable without confusing it with the bridge's own LiveKit room session.

## Concurrency and lifecycle

### `queue`

Thread-boundary handoff that ensures node-affine work runs on the correct thread.

Example:

- `RosExecutorQueue`

### `drain`

Execute or empty queued work on the owning thread.

Example:

- `RosExecutorQueue::drain()`

### `quiesce`

Stop admitting new entries into a callback or critical scope while waiting for active entrants to finish.

Examples:

- `QuiesceGate`
- `ReentrantQuiesceGate`

### `ensureRunning`

Idempotent initialization or start of a runtime object that may already be active.

Examples:

- `VideoStreamInstance::ensureRunning()`
- `VideoFrameSource::ensureRunning()`

## Practical rules

- Use `config` for declared parameters and `spec` for resolved runtime structure.
- Use `track` for LiveKit transport and `stream` for the bridge's end-to-end runtime.
- Use `cdr` only for serialized ROS payload bytes.
- Use `source` only when there is a real input-side runtime abstraction.
- Reserve `publisher` for ROS -> LiveKit publication ownership.
- Prefer `writer` for LiveKit -> ROS ingress execution.
- Reserve `request` and `response` for RPCs.
- Prefer `command`, `heartbeat`, `status`, `entry`, or `update` for fire-and-forget control paths.
- Use `resource` only for actual ROS graph entities.
- Use `target` for caller-selected identifiers that may include non-ROS configured sources.
- Avoid naked `session` for requester/browser lifecycle state in new internal names.

## Current migration notes

- Preferred ingress name: `RosTopicWriter`
- Legacy ingress name to phase out: `RosTopicPublisher`
- `Publisher` remains reserved for ROS -> LiveKit names such as `DataTrackPublisher` and `VideoTrackPublisher`
- Mixed topic/configured-source state should move toward `target_*` naming instead of bare `resource`
- New internal names for requester/browser lifecycle should qualify the meaning instead of introducing more bare `session` names
