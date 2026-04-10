# Documentation map

The repository README is the fast path for deciding whether this package fits and getting to a first run. This folder holds the deeper docs for integrators, operators, and source readers.

| If you need to... | Read this |
| --- | --- |
| Understand how a client should use the bridge | [integration.md](./integration.md) |
| Understand the LiveKit-facing contract | [protocol.md](./protocol.md) |
| Implement heartbeat and stream handling correctly | [subscriptions.md](./subscriptions.md) |
| Configure auth, access rules, publish limits, and video entries | [runtime-configuration.md](./runtime-configuration.md) |
| Understand allow and deny behavior | [access-control.md](./access-control.md) |
| Configure ROS-backed and external video sources | [video-sources.md](./video-sources.md) |
| Read the runtime from the source outward | [runtime-architecture.md](./runtime-architecture.md) |
| Run or extend the test suite | [testing.md](./testing.md) |
| Debug startup, runtime, or test failures | [troubleshooting.md](./troubleshooting.md) |

## Reading paths

If you are building a LiveKit client, start with [integration.md](./integration.md), then move to [protocol.md](./protocol.md), [subscriptions.md](./subscriptions.md), and [access-control.md](./access-control.md).

If you are enabling video delivery, read [video-sources.md](./video-sources.md), [runtime-configuration.md](./runtime-configuration.md), and [troubleshooting.md](./troubleshooting.md).

If you are reading or changing the implementation, start with [runtime-architecture.md](./runtime-architecture.md), then [subscriptions.md](./subscriptions.md), and [testing.md](./testing.md).
