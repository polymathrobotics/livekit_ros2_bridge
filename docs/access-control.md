# Access control

The bridge uses one policy model everywhere it authorizes ROS resources:

- normalize the requested name first
- evaluate the operation-specific allow and deny rules
- let deny rules win over allow rules
- reject the request if nothing allows the normalized name

The policy is operation-specific, not requester-specific. Requester identity still matters for authentication, session handling, and quotas, but not for name matching.

## Which rules apply where

| Operation | Parameters | Surfaces |
| --- | --- | --- |
| Publish | `access.rules.publish.allow`, `access.rules.publish.deny` | `ros.topics.publish` |
| Subscribe | `access.rules.subscribe.allow`, `access.rules.subscribe.deny` | topic entries in `ros.subscriptions.heartbeat`, `ros.topics.list` |
| Service | `access.rules.service.allow`, `access.rules.service.deny` | `ros.services.call`, `ros.services.list` |

`configured_source` video targets are the exception. They are controlled by which `video_configured_source_ids` and `video.configured_sources.*` entries exist, not by `access.rules.subscribe.*`.

## Name normalization and patterns

All policy checks normalize names before matching:

- trim surrounding whitespace
- collapse repeated `/`
- add a leading `/` when missing
- remove a trailing `/` unless the name is `/`

Examples:

- `cmd_vel` becomes `/cmd_vel`
- ` /robot//camera/ ` becomes `/robot/camera`
- blank or whitespace-only names normalize to empty and are denied

Configured rules are normalized the same way. Empty rule entries are ignored.

Pattern syntax is intentionally small:

- `*` matches all resources for that operation
- `/foo/bar` matches exactly `/foo/bar`
- `/foo/*` matches descendants under `/foo`, including `/foo/bar`

There is no other glob syntax.

## Rule precedence

Each operation has its own allow set and deny set. Evaluation order is:

1. If the deny set contains `*` or a matching pattern, reject.
2. Otherwise, if the allow set contains `*`, allow.
3. Otherwise, if the allow set is empty, reject.
4. Otherwise, allow only if an allow pattern matches.

That makes the policy default-deny, even when the parameter exists but the allow list is empty.

## Surface-specific behavior

### Publish

`ros.topics.publish` is checked against `access.rules.publish.*`.

Important details:

- anonymous publish packets are dropped before policy checks
- the request topic is normalized during parsing
- the requested `interface_type` must match the bridge's resolved topic type exactly
- once the bridge has created a cached publisher for a topic, later requests are checked against that cached type instead of re-reading the ROS graph

### Subscribe

Topic subscriptions from `ros.subscriptions.heartbeat` are checked against `access.rules.subscribe.*` before the bridge renews the lease.

Important details:

- a forbidden topic becomes a per-subscription `forbidden` error in `ros.subscriptions.status`
- `ros.topics.list` uses the same subscribe policy, so callers only see topics they could subscribe to
- `configured_source` targets are trimmed and resolved through `video.configured_sources.*`, but they do not use subscribe rules

For the heartbeat and status contract, read [subscriptions.md](./subscriptions.md).

### Service

`ros.services.call` and `ros.services.list` both use `access.rules.service.*`.

Important details:

- anonymous RPC calls are rejected before request parsing
- the service name is normalized before the policy check
- `ros.services.list` filters the ROS graph through the same rules as `ros.services.call`
- service authorization is about the service name only; interface resolution is a separate step

## Interface resolution by surface

The bridge resolves interface types differently depending on the surface:

| Surface | Name source | Interface-type source |
| --- | --- | --- |
| `ros.topics.publish` | normalized request topic | the unique graph type, or the cached publisher type if the bridge already created one |
| topic subscription heartbeat | normalized request topic | exactly one graph-advertised topic type |
| `ros.services.call` | normalized request service | caller-supplied `interface_type`, otherwise exactly one graph-advertised service type |
| `ros.topics.list` and `ros.services.list` | ROS graph entries | only resources with exactly one interface type are returned |

The bridge does not guess when the graph is ambiguous. List responses skip ambiguous resources, topic subscriptions fail, and topic publish rejects the request.
