# Runtime architecture

`Runtime` is the bridge's top-level owner for one ROS node and one `RoomSession`. It eagerly constructs the ROS-facing helpers, starts the LiveKit session, registers the RPC surface, and then spends the rest of its life moving LiveKit callbacks onto executor-affine ROS work.

The important boundary is simple:

- LiveKit callbacks and reconnect logic run outside the ROS executor
- ROS graph access, topic publication, subscription maintenance, and service request creation are funneled back through `RosExecutorQueue`

## Main owners

`Runtime` owns the long-lived pieces that need coordinated teardown:

- `RoomSession`: background connect and reconnect loop plus the active LiveKit room
- `RosExecutorQueue`: custom waitable that wakes the ROS executor and drains queued work on that executor thread
- `RosServiceCaller`: dynamic ROS service clients plus a short poll timer that settles pending responses
- `SubscriptionRegistry`: shared lease state, data-track republish bookkeeping, and video stream bindings
- `VideoStreamRegistry`: one shared in-process video runtime per resolved stream key
- `RosTopicPublisher`: best-effort topic ingress with a bounded publisher cache
- `RpcRouter` and `ControlPacketRouter`: the LiveKit-facing entry points

## Event flow

Most LiveKit-originated events enter through `RoomSessionCallbacks`.

The flow looks like this:

1. `RoomSession` receives a control packet, RPC, disconnect event, or reconnect event.
2. `Runtime::submitExecutorWork()` queues executor-affine work onto `RosExecutorQueue`.
3. `RosExecutorQueue::drain()` runs that work on the ROS executor thread.
4. ROS-facing helpers do the actual graph lookups, topic publication, lease renewal, or service request creation.

`submitExecutorWork()` rejects new work once shutdown starts, and it checks again at execution time in case shutdown raced with enqueue.

## Startup order

Construction is eager rather than lazy:

1. `Runtime` builds `RosExecutorQueue`, `DataTrackPublisher`, `RosTopicPublisher`, `SubscriptionRegistry`, `SubscriptionHeartbeatProcessor`, `RosServiceCaller`, `RpcRouter`, and `ControlPacketRouter`.
2. It creates a one-second lease GC timer. That timer also hops back through `submitExecutorWork()`.
3. It starts `RoomSession` with callbacks for session reset, participant disconnect, and incoming control packets.
4. After the session thread is running, it registers the LiveKit RPC methods.

That order matters. The ROS-side helpers exist before the session can emit callbacks, and the RPC surface is not exposed until the runtime has everything needed to serve those calls.

## Reconnect and reset behavior

`RoomSession` runs one background loop:

- try to connect once
- wait for disconnect or stop
- clear per-connection room state
- reconnect with exponential backoff unless stop was requested

The reset contract is per connection, not per `Runtime` instance:

- `on_session_reset` unpublishes data tracks, resets `SubscriptionRegistry`, and fails pending service calls
- participant disconnect callbacks are suppressed during transport reconnect so transient reconnects do not look like permanent departures
- when a requester really disconnects outside reconnect handling, the bridge keeps the lease state, marks that requester for data-track republish, and cancels only that requester's pending service calls

That is why browser refresh can keep lease state alive while still forcing data-track publications to be rebuilt for the new participant session.

## Why service calls span two phases

`ros.services.call` crosses both sides of the runtime boundary.

Phase 1 happens on the ROS executor:

- resolve the service type
- create or reuse the `rcl` service client
- deserialize the request
- call `rcl_send_request()`

Phase 2 happens later on `RosServiceCaller`'s poll timer:

- call `rcl_take_response()`
- match responses by client pointer and sequence number
- fulfill the stored promise
- time out or cancel pending calls when needed

That split keeps executor-affine request creation safe without blocking the executor until the remote service replies.

Immediate failures such as shutdown, bad requests, quota limits, or client creation errors fail at phase 1. Later failures such as timeout, requester disconnect, session reset, or shutdown settle the stored promise at phase 2.

## Shutdown order

`Runtime::shutdown()` is idempotent and tears down in dependency order:

1. flip the shutdown flag so new work is dropped
2. stop the lease GC timer
3. unregister RPC methods from the active session
4. stop `RoomSession` so no new SDK callbacks can enqueue ROS work
5. shut down `RosExecutorQueue`
6. shut down `SubscriptionRegistry`, unpublish data tracks, stop video streams, shut down `RosServiceCaller`, and clear topic publishers

The key invariant is that the session stops before the executor queue is torn down. Already accepted work may still be draining at that point, so the runtime also checks the shutdown flag inside queued lambdas.
