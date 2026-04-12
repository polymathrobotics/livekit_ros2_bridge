# Naming guide

This document sets down the ubiquitous language used within `livekit_ros2_bridge`.

These are project-local definitions. Some of these words have broader meanings elsewhere. When
reading or changing this codebase, use the meanings in this document.

This guide applies to:

- C++ type and function names
- internal documentation
- code comments
- test names and helpers
- log and event naming when describing stable implementation concepts

This guide does not define the wire contract. When wire protocol terms or payload shapes differ,
use [protocol.md](./protocol.md) and [subscriptions.md](./subscriptions.md) for the external
contract and this guide for internal implementation vocabulary.

## Terminology

### Layer terms

- **Wire path**
  - `request` / `response`
    - RPC-style interactions that expect a paired response.
  - `command`
    - A fire-and-forget execution request.
  - `heartbeat`
    - A lease-renewal envelope.
  - `demand`
    - One lease-backed subscription declaration carried by a heartbeat.
  - `status`
    - The bridge's reported outcome on a control surface.

- **Runtime ownership**
  - `config`
    - Declared startup configuration loaded from parameters.
    - Not request-specific.
    - Treated as immutable during runtime.
    - Examples: `RuntimeConfig`, `VideoStreamConfig`, `RosVideoTopicRule`, `ConfiguredVideoStreamSource`
  - `spec`
    - Resolved runtime inputs produced from config plus a specific request or lookup result.
    - Carries concrete values the runtime can execute.
    - Example: `VideoStreamSpec`
  - `registry`
    - Central manager and lifecycle owner for shared, deduplicated resources.
    - Examples: `SubscriptionRegistry`, `VideoStreamRegistry`
  - `instance`
    - One live shared logical runtime tying an input to an output.
    - Examples: `DataStreamInstance`, `VideoStreamInstance`

### Directionality and transport

- **`publisher`**
  - Reserved for ROS -> LiveKit publication ownership.
  - A publisher owns or maintains a LiveKit publication.
  - Does not mean LiveKit -> ROS ingress.
  - Examples: `DataTrackPublisher`, `VideoTrackPublisher`

- **`writer`**
  - Preferred for LiveKit -> ROS best-effort writes.
  - A writer executes ingress commands into ROS.
  - Distinct from a LiveKit publication owner.
  - Example: `RosTopicWriter`

- **`track`**
  - LiveKit transport object or deterministic LiveKit transport surface only.
  - Examples: `livekit::LocalDataTrack`, `PublishedVideoTrack`, `track_name`

- **`stream`**
  - The bridge's end-to-end logical delivery runtime.
  - Broader than a LiveKit track.
  - May include ROS subscriptions, GStreamer pipelines, leases, and republish behavior.
  - Examples: `DataStreamInstance`, `VideoStreamInstance`, `stream_key`

- **`payload`**
  - Message envelope or serialization helper.
  - Use for bytes and envelope parsing, not for transport or lifecycle concepts.

- **`cdr`**
  - Payload encoding only.
  - Describes serialized ROS bytes and never the transport or delivery surface.

### Data path

- **Data pipeline**
  - `instance`
    - One live logical data stream per normalized ROS topic.
  - `subscription`
    - The ROS `rclcpp::GenericSubscription` input side.
  - `publisher`
    - One LiveKit data-track owner.
  - `payload`
    - Message envelope and serialization helpers.
  - `cdr`
    - Serialized ROS bytes only.
  - `track`
    - The LiveKit data transport object or name.
  - `stream`
    - The full runtime from ROS subscription through LiveKit delivery.
  - Examples: `DataStreamInstance`, `DataTrackPublisher`

- **Video pipeline**
  - `config`
    - Declared video configuration.
  - `spec`
    - Resolved video stream input and publish settings.
  - `instance`
    - One live shared video runtime per `stream_key`.
  - `source`
    - Frame-producing input runtime.
  - `publisher`
    - One LiveKit video-track owner.
  - `track`
    - The LiveKit video transport object or name.
  - `stream`
    - The logical video delivery runtime.
  - Examples:
    - `RosVideoTopicRule`
    - `ConfiguredVideoStreamSource`
    - `VideoStreamSpec`
    - `VideoFrameSource`
    - `VideoTrackPublisher`

- **`source`**
  - A real input-side runtime abstraction that produces frames or messages for another component.
  - Do not use `source` for a mere identifier, config entry, or upstream system unless there is an
    actual producing runtime behind it.
  - Example: `VideoFrameSource`

### Control and RPC terms

- **RPC vocabulary**
  - `request` / `response`
    - Reserved for RPC-style interactions that expect a paired response.
    - Examples: `ServiceCallRequest`, `ServiceCallResponse`
  - `invocation`
    - One inbound RPC execution context.
    - Example: `RpcInvocation`
  - `caller`
    - The component executing a ROS service request.
    - Example: `RosServiceCaller`
  - `inflight`
    - Unresolved quota-tracked RPC work.

- **Control-path vocabulary**
  - `command`
    - Fire-and-forget execution request.
    - Example: `TopicPublishCommand`
  - `heartbeat`
    - Lease-renewal envelope.
    - Example: `SubscriptionHeartbeat`
  - `demand`
    - One lease-backed subscription declaration carried by a heartbeat.
    - A demand names one requested subscription target plus any delivery preferences for that target.
    - Multiple demands for the same canonical target aggregate into one shared runtime and one
      applied delivery state.
    - Example: `SubscriptionDemand`
  - `status`
    - The bridge's reported control-path outcome.
    - Example: `SubscriptionStatus`
  - `update`
    - Fire-and-forget control payload that mutates or refreshes state without RPC semantics.

- **`router`**
  - Distributes inbound LiveKit traffic to the appropriate internal handler.
  - A router decides where work goes; it does not own the downstream runtime state.
  - Examples: `RpcRouter`, `ControlPacketRouter`

### Access control and lookup

- **`policy`**
  - The top-level authorization manager.
  - Example: `AccessPolicy`

- **`operation`**
  - The action being authorized.
  - Examples: `AccessOperation::Publish`, `AccessOperation::Subscribe`, `AccessOperation::CallService`

- **`rule`**
  - One allow or deny matching rule inside a policy.

- **`resource`**
  - A real ROS graph entity normalized for access checks and lookup helpers.
  - Use `resource` only when the value must be a ROS graph entity.
  - Examples: ROS topic names, ROS service names

- **`target`**
  - What the caller asked for on a control or subscription surface.
  - A target may resolve to a ROS resource, but it does not have to.
  - Use `target` when the value may be either a ROS topic or a configured source.
  - Examples: a topic subscription target, a `configured_source` target
  - For mixed topic and configured-source runtime state, prefer names like `target_name`,
    `target_kind`, or `target_key` instead of a generic `resource` field.

### Connections, sessions, and leases

- **`connection`**
  - The bridge-owned LiveKit room transport lifecycle.
  - One active or reconnecting attachment between the bridge and a LiveKit room.
  - Owns transport-scoped state such as the active room handle, registered RPC methods, and
    published track bindings that must be rebuilt after reconnect.
  - Reconnect tears down one connection and establishes another.
  - Examples: `RoomConnection`, `RoomConnectionCallbacks`, `makeRoomConnection()`,
    `FakeRoomConnection`, `on_connection_reset`

- **`session`**
  - Actor-scoped logical lifetime, never the bridge's room transport.
  - A session belongs to a requester or participant identity, not to the bridge transport.
  - Sessions can outlive or be replaced independently of stream leases and independently of the
    bridge's room connection.
  - Do not use bare `Session` or bare `session` for new internal names unless the code is directly
    mirroring the wire field name `session_id`.
  - Use `client_session` or `participant_session` when session semantics are required.
  - Examples: `client_session_id`, `ClientSessionLease`, `participant_session`

- **`lease`**
  - Time-bound ownership or identity binding.
  - A lease is not a transport connection and is not itself a participant incarnation.
  - Leases apply to shared stream ownership and to client-session fallback bindings.
  - Examples: requester lease, stream lease, client session lease

### Concurrency and lifecycle

- **`queue`**
  - Thread-boundary handoff that ensures node-affine work runs on the correct thread.
  - Example: `RosExecutorQueue`

- **`drain`**
  - Execute or empty queued work on the owning thread.
  - Example: `RosExecutorQueue::drain()`

- **`quiesce`**
  - Stop admitting new entries into a callback or critical scope while waiting for active entrants
    to finish.
  - Examples: `QuiesceGate`, `ReentrantQuiesceGate`

- **`ensureRunning`**
  - Idempotent initialization or start of a runtime object that may already be active.
  - Examples: `VideoStreamInstance::ensureRunning()`, `VideoFrameSource::ensureRunning()`

## Practical rules

- Use `config` for declared parameters and `spec` for resolved runtime structure.
- Use `track` for LiveKit transport and `stream` for the bridge's end-to-end runtime.
- Use `cdr` only for serialized ROS payload bytes.
- Use `source` only when there is a real input-side runtime abstraction.
- Reserve `publisher` for ROS -> LiveKit publication ownership.
- Prefer `writer` for LiveKit -> ROS ingress execution.
- Reserve `request` and `response` for RPCs.
- Prefer `command`, `heartbeat`, `demand`, `status`, or `update` for control paths.
- Use `resource` only for actual ROS graph entities.
- Use `target` for caller-selected identifiers that may include non-ROS configured sources.
- Use `connection` for bridge transport state, `session` only for qualified actor lifetimes, and
  `lease` for time-bound ownership or identity binding.
