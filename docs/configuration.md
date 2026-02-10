# Configuration reference

`livekit_bridge` is configured via ROS parameters (usually from a YAML params file).

## Params file shape

Your params file must be keyed by the node name:

```yaml
livekit_bridge:
  ros__parameters:
    # parameters here...
```

The packaged launch file loads the YAML with substitutions enabled (`$(env ...)`, etc).

## Layering params files (recommended)

The packaged launch file supports parameter overlays:

- `params_file` (base) is loaded first (defaults)
- `overlay_params_file` is loaded second (overrides)

Later params override earlier ones. This lets you keep using the packaged defaults in
`config/livekit_bridge.yaml` and provide a small per-deployment overlay (typically just allowlists).

Template overlay file (copy and edit): `config/livekit_bridge.overlay.yaml`.

## LiveKit connection and auth

Required:

- `livekit.url` (string)
- `livekit.room` (string)

Optional:

- `livekit.identity` (string; default: `${node_name}-${hostname}`)
- `livekit.token` (string; static token)
- `livekit.api_key` (string; optional)
- `livekit.api_secret` (string; optional)
- `livekit.token_ttl_seconds` (int; default: `3600`; used only for API key/secret token minting)

Auth selection:

- Use `livekit.token`, or
- Use `livekit.api_key` + `livekit.api_secret` (the bridge mints a token), or
- Error if neither is provided

The packaged `config/livekit_bridge.yaml` is wired to `LIVEKIT_URL`, `LIVEKIT_ROOM`, `LIVEKIT_IDENTITY`, and either `LIVEKIT_TOKEN` or `LIVEKIT_API_KEY` + `LIVEKIT_API_SECRET`.

## Reconnect tuning

- `livekit.reconnect.initial_backoff_ms` (int; default: `500`)
- `livekit.reconnect.max_backoff_ms` (int; default: `10000`)

## Bridge access control (required)

The bridge is **default-deny**:

- If an allowlist is empty, that operation is denied.
- The node refuses to start if *all* allowlists are empty (when using the default static access rules).

Allow/deny lists:

- `access.static.publish.allow` / `access.static.publish.deny` (string array)
- `access.static.subscribe.allow` / `access.static.subscribe.deny` (string array)
- `access.static.service.allow` / `access.static.service.deny` (string array)

Notes:

- Denylists override allowlists.
- `*` is treated specially only in allowlists. If you put `*` in a denylist, it effectively denies everything.

### Pattern language

Allow/deny entries are matched against normalized ROS names (leading `/`, duplicate slashes collapsed, trailing `/` removed).

Supported patterns:

- Exact match: `/user/cmd_vel`
- Prefix match: `/user/*` matches `/user/foo` and `/user/bar/baz` (but not `/user`)
- Allow all (allowlist only): `*`

Notes:

- There is no general globbing (`?`, `**`, regex, etc.).
- To allow a prefix *and* the parent name, include both `/user` and `/user/*`.

### Subscription type denylist

- `access.static.subscribe.type_deny` (string array)

Defaults block:

- `sensor_msgs/msg/Image`
- `sensor_msgs/msg/CompressedImage`
- `sensor_msgs/msg/PointCloud2`

## Limits

Subscriptions:

- `subscribe.max_total_subscriptions` (int; default: `128`; `0` disables the total-subscription limit)
- `subscribe.max_subscriptions_per_participant` (int; default: `32`; `0` disables the per-participant limit)
- Topics with multiple ROS types are rejected.

Publishing:

- `publish.max_topics` (int; default: `50`)
- LiveKit→ROS publish payloads are accepted only when the ROS graph reports exactly one type for the topic and it matches
  the payload `type`. This helps prevent type poisoning by remote clients.

Services:

- `service.timeout_ms` (int; default: `2000`)
- `service.max_inflight_per_participant` (int; default: `4`)

## Example config

This is a complete example you can adapt:

```yaml
livekit_bridge:
  ros__parameters:
    # Connection
    livekit.url: "$(env LIVEKIT_URL)"
    livekit.room: "$(env LIVEKIT_ROOM)"
    livekit.identity: "$(env LIVEKIT_IDENTITY '')"

    # Auth (choose one)
    livekit.token: "$(env LIVEKIT_TOKEN '')"
    # livekit.api_key: "$(env LIVEKIT_API_KEY '')"
    # livekit.api_secret: "$(env LIVEKIT_API_SECRET '')"

    # Policy (required: set at least one allowlist)
    access.static.publish.allow: ["/user/cmd_vel"]
    access.static.subscribe.allow: ["/user/*"]
    access.static.service.allow: ["/example/service"]
```

## Extension hooks (optional)

Factory-based extension hooks are configured with:

- `access.factory` (string; default: empty)
- `telemetry.factory` (string; default: empty)

See [extensions.md](extensions.md) for symbol-path format, factory contracts, and validation behavior.
