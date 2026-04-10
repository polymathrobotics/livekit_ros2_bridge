# Runtime configuration

The bridge reads ROS parameters once at node startup and builds one immutable runtime snapshot. Reconnects reuse that snapshot. Changing parameters later does not change live behavior.

At startup, configuration determines:

- how the bridge connects to LiveKit and identifies itself
- whether the bridge reuses a static token or mints tokens with API credentials
- which ROS resources each access policy family exposes
- how video requests resolve to ROS-backed or configured external sources
- whether bridge-managed sidecars are available

If a change would affect any of those decisions, restart the node. Reconnect alone is not enough.

## Required connection settings

| Parameter | Required | Notes |
| --- | --- | --- |
| `livekit.url` | yes | LiveKit server URL |
| `livekit.room` | yes | Room name the bridge should join |
| `livekit.identity` | no | Defaults to `<node_name>-<hostname>`, or just `<node_name>` if hostname lookup fails |

Startup fails if `livekit.url` or `livekit.room` is empty.

## Authentication modes

Valid startup shapes:

| Parameters | Bridge token source | Sidecars available |
| --- | --- | --- |
| `livekit.token` | reuse the configured token as-is | no |
| `livekit.api_key` + `livekit.api_secret` | mint bridge tokens at connect time | yes |
| `livekit.token` + `livekit.api_key` + `livekit.api_secret` | reuse the configured token as-is | yes |

Invalid startup shapes:

- neither `livekit.token` nor a full `livekit.api_key` + `livekit.api_secret` pair is set
- only one of `livekit.api_key` or `livekit.api_secret` is set

When the bridge mints its own token, it enables the LiveKit room grants it needs to join, publish, subscribe, and publish data.

## Token lifetime and refresh

These parameters matter only when API credentials are in play:

- `livekit.token_ttl_seconds`
- `livekit.token_refresh_margin_seconds`

Rules:

- `livekit.token_ttl_seconds` must be greater than `0` when API credentials are used
- `livekit.token_refresh_margin_seconds` is the lead time before expiry that triggers bridge reconnect or sidecar restart
- API-minted bridge tokens are refreshed by reconnecting before expiry
- static bridge tokens are never refreshed automatically
- for static tokens, the bridge only parses the JWT `exp` claim well enough to log an expiry warning; it does not verify the signature as part of that check
- sidecar refresh lead time is clamped to at most half of the token TTL so a large margin does not cause immediate restart churn

## Access policy parameters

Access rules are also startup-only:

- `access.rules.publish.allow`
- `access.rules.publish.deny`
- `access.rules.subscribe.allow`
- `access.rules.subscribe.deny`
- `access.rules.service.allow`
- `access.rules.service.deny`

Empty allow lists are valid, but they deny everything for that operation. Deny rules always win over allow rules.

For matching behavior, read [access-control.md](./access-control.md).

## Publish cache limit

`publish.max_topics` controls how many active ROS publishers the bridge keeps cached for `ros.topics.publish`.

Important behavior:

- default: `50`
- `0` means no limit
- the bridge creates generic publishers on demand
- when the cache grows past the limit, the least recently used cached publisher is evicted after the current publish succeeds

## Video entries

Video entries are defined under `videos.*` and loaded only at startup.

Common parameters:

| Parameter | Meaning |
| --- | --- |
| `videos.ids` | the entry ids to load |
| `videos.<id>.kind` | `ros` or `pipeline` |
| `videos.<id>.pattern` | topic pattern for `ros` entries |
| `videos.<id>.pipelines` | `alias=pipeline` strings |

Rules by kind:

- `kind: ros` supports the pipeline aliases `image`, `compressed_image`, and `default`
- `kind: pipeline` supports only the `default` alias
- duplicate `videos.ids` entries are rejected
- configured source ids normalize to the external names clients request

Examples:

- `videos.front_camera.kind: ros` matches ROS topics according to `videos.front_camera.pattern`
- `videos.front_rtsp.kind: pipeline` creates a configured external source requested as `external: "/front_rtsp"`

The bridge only creates the sidecar supervisor when API credentials are available. That means:

- a static `livekit.token` is enough for the bridge itself to join LiveKit
- the same config cannot launch bridge-managed video sidecars unless `livekit.api_key` and `livekit.api_secret` are also set

## Runtime image

The repo runtime image uses the same parameter file shape as a workspace run.

By default it reads `/config/livekit_bridge.params.yaml`. Set `LIVEKIT_BRIDGE_PARAMS_FILE` to use a different in-container path.

The runtime image also exports `GST_PLUGIN_PATH` for the installed `gstrosbridge` plugin, so built-in ROS video source readers do not need extra plugin-path setup there.

For rule resolution and sidecar behavior, read [video-sources.md](./video-sources.md). For the LiveKit-facing contract, read [protocol.md](./protocol.md) and [subscriptions.md](./subscriptions.md).
