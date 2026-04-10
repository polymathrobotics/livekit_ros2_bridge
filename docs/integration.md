# Integration guide

This page is for people building a browser, mobile, or backend client that talks to the bridge over LiveKit. Read it after the README, then move to [protocol.md](./protocol.md) when you need exact request and response shapes.

The bridge keeps the ROS graph private. Your client talks only to the fixed LiveKit-facing contract, and the bridge decides which ROS resources are visible based on startup configuration.

## Mental model

The bridge uses three kinds of LiveKit surfaces:

- RPCs for request-response flows such as listing resources, fetching interface definitions, and calling services
- control topics for best-effort commands and subscription lease renewal
- tracks for ongoing delivery of ROS topic data or video

That separation matters when you design a client:

- use RPCs when you need a direct success or failure result
- use `ros.topics.publish` for small best-effort topic ingress
- use `ros.subscriptions.heartbeat` to declare the full set of streams you still want
- consume `ros.subscriptions.status` and the named LiveKit tracks to understand what the bridge actually made available

## Typical client flow

Most integrations follow this order:

1. Join the same LiveKit room as the bridge.
2. Call `ros.topics.list` and `ros.services.list` to discover only the resources your policy allows.
3. Call `ros.interfaces.get` for the message and service types your client needs to encode or decode.
4. Use `ros.services.call` for request-response operations.
5. Send `ros.topics.publish` packets for small allowed topic writes.
6. Send `ros.subscriptions.heartbeat` on a regular cadence to request topic or video streams.
7. Read `ros.subscriptions.status` to learn whether each requested stream is active, forbidden, unavailable, or not found.
8. Subscribe to the announced LiveKit data track or video publication.

For a first integration, start with one service call or one topic subscription path. Once that works, add more interface types, video, and broader policy rules.

## Choosing the right surface

| Need | Surface |
| --- | --- |
| Discover allowed topics | `ros.topics.list` |
| Discover allowed services | `ros.services.list` |
| Fetch `.msg` or `.srv` definitions | `ros.interfaces.get` |
| Call a ROS service | `ros.services.call` |
| Publish a small ROS message | `ros.topics.publish` |
| Request topic or video delivery | `ros.subscriptions.heartbeat` |
| Learn current subscription state | `ros.subscriptions.status` |

Two practical boundaries are easy to miss:

- topic publish is best-effort and does not send an acknowledgement packet
- subscription heartbeats are lease renewals, not one-time start commands

## Identities, leases, and access

The bridge treats LiveKit identity as part of the contract:

- RPCs require `caller_identity`
- `ros.topics.publish` packets from anonymous callers are dropped
- subscription heartbeats prefer `requester_identity`, with `session_id` used only as a narrow fallback when LiveKit omits that identity from user-data packets

Subscriptions are requester-scoped leases:

- each heartbeat says "this is the full set of streams I still want"
- the bridge renews requested streams for 45 seconds
- if a requester stops sending heartbeats, that requester's leases eventually expire
- if multiple requesters share a stream, the bridge keeps one shared runtime resource alive until the last lease disappears

Authorization is name-based, not requester-specific. A caller identity must be present, but allow and deny rules are evaluated against normalized resource names loaded at startup. For the exact matching rules, see [access-control.md](./access-control.md).

## Topic data versus video

The bridge has two delivery modes:

- non-video ROS topics are delivered as raw CDR bytes on a LiveKit data track
- ROS image topics and configured `external` sources are delivered as video through a managed sidecar publisher

This means your client needs different expectations for each:

- data-track subscriptions need interface definitions from `ros.interfaces.get`
- video subscriptions depend on `videos.*` configuration and, for bridge-managed sidecars, LiveKit API credentials

See [video-sources.md](./video-sources.md) for the rules that decide which path a stream uses.

## Where to go next

- [protocol.md](./protocol.md) for exact request and response shapes
- [subscriptions.md](./subscriptions.md) for lease timing, status payloads, and replay behavior
- [runtime-configuration.md](./runtime-configuration.md) for deployment and auth setup
- [troubleshooting.md](./troubleshooting.md) for operational debugging
