# Testing

Most of the suite is small-unit `gtest`, but the more valuable maintenance coverage lives in the integration-style tests under `test/cpp/`. Those tests build a real `rclcpp::Node`, attach a `SingleThreadedExecutor`, and replace the LiveKit edge with `FakeRoomSession`.

## Run tests

From the repository root:

```bash
just test
```

The `just` wrapper builds first, then runs the suite inside the repository's dev-container workflow.

To run the same `colcon` unit-test suite across every supported ROS distro with
ephemeral Docker builds:

```bash
just test-matrix
```

You can scope the matrix to specific distros:

```bash
just test-matrix jazzy rolling
```

That path uses the `docker buildx bake` `test-*` targets, so each run starts
from a clean image instead of reusing the stateful local dev container. It is
also the same cross-distro path used by CI.

To narrow to one `ament_add_gtest` target:

```bash
just test --ctest-args -R test_runtime
just test --ctest-args -R test_subscription_heartbeat_processor
```

`CMakeLists.txt` registers one test target per `test/cpp/test_*.cpp` file, so the filter usually matches the filename stem.

## What the suite covers

Good entry points:

- `test_runtime.cpp`: end-to-end runtime wiring around `FakeRoomSession`
- `test_subscription_heartbeat_processor.cpp`: heartbeat parsing, status envelopes, access control, and video-source edge cases
- `test_subscription_registry.cpp`: lease sharing, interval handling, CDR replay, and video stream interactions
- `test_rpc_router.cpp`: RPC request parsing, graph filtering, and error mapping
- `test_runtime_config.cpp`: auth modes, parameter validation, and video rule/source loading
- `test_video_stream_manager.cpp`: ROS ingress, external pipelines, and global video publish settings

## Common harness pattern

Reusable test helpers live in `test/cpp/ros_test_support.hpp` and `test/cpp/fake_room_session.hpp`.

The common pattern is:

1. Initialize ROS once for the suite with `ScopedRclcppInit`.
2. Create a fresh node per test.
3. Add the node to a `rclcpp::executors::SingleThreadedExecutor` when the test depends on graph discovery, timers, subscriptions, or queued executor work.
4. Construct the component under test.
5. Drive asynchronous behavior with polling helpers instead of fixed sleeps.

Useful helpers:

- `spinUntil(...)`: advance the executor with `spin_some()` while waiting for a predicate
- `waitUntil(...)`: wait for non-ROS state changes such as files, promises, or counters
- `waitForTopicType(...)`: wait for graph visibility before a test tries to publish or subscribe

If `waitForTopicType(...)` times out, it logs `event=wait_for_topic_type_timeout`, which is usually the first useful clue.

## When to use `FakeRoomSession`

`FakeRoomSession` is more than a transport stub. Its `state` object is the assertion surface.

Use it when you need to verify:

- which RPC methods were registered or unregistered
- which control packets were published, and to which recipients
- which CDR tracks were published, unpublished, or rejected
- which callbacks `Runtime` installed
- how reconnect-like events behave when injected with `emitSessionReset()`, `emitParticipantDisconnected()`, or `emitIncomingControlPacket(...)`

Configure the fake before constructing `Runtime` or other owners. Several startup and teardown behaviors read `FakeRoomSessionState` only once.

## A reliable async rule

If a test is flaky, replace ad hoc sleeps first.

- use `spinUntil(...)` when executor progress is required
- use `waitUntil(...)` when the state change happens outside ROS callbacks

That matters because many bridge operations cross thread boundaries: executor queue handoff, ROS graph discovery, data-track publication, video stream management, and service-call completion.
