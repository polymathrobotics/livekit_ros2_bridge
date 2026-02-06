# Extension Hooks

This page documents the optional extension system used by `livekit_ros2_bridge/core/extensions.py` and the
`access.factory` / `telemetry.factory` ROS parameters.

## Symbol loader (`load_symbol`)

`load_symbol(path)` dynamically imports a Python symbol from either form:

- `module:attr`
- `module.attr`

Nested attribute paths are supported in the attribute portion:

- `json:decoder.JSONDecoder`

Behavior and validation:

- Leading/trailing whitespace is ignored.
- Empty paths raise `ValueError`.
- Paths that do not include a module plus attribute raise `ValueError`.
- Empty attribute segments (for example `json..loads` or `json:decoder.`) raise `ValueError`.
- Missing attributes raise `ImportError`.
- Module import failures propagate from `importlib.import_module(...)`.

## Custom access factory

Parameter:

- `access.factory` (string; default: empty)

When set, the bridge resolves the symbol, verifies it is callable, and calls:

```py
access_policy = factory(node=<LivekitBridgeNode>, config=<AccessPolicyConfig>)
```

The returned object must provide:

- `authorize(ctx, op, res)` (the `AccessPolicy` contract)

Notes:

- With a custom access factory, the static allowlist startup check is skipped.
- Your custom access policy is responsible for enforcing any default-deny behavior you require.

## Custom telemetry factory

Parameter:

- `telemetry.factory` (string; default: empty)

When set, the bridge resolves the symbol, verifies it is callable, and calls:

```py
telemetry = factory(node=<LivekitBridgeNode>, config=<RuntimeConfig>)
```

The returned object must provide:

- `emit(ctx, event)` (the `Telemetry` contract)

`ctx` is `RequestContext | None` (`None` is used for non-request-scoped events, like egress message telemetry).

`event` is one of:

- `RpcTelemetryEvent` (`kind == "rpc"`)
- `AccessDenyTelemetryEvent` (`kind == "access_deny"`)
- `IngressPublishTelemetryEvent` (`kind == "ingress_publish"`)
- `EgressMessageTelemetryEvent` (`kind == "egress_message"`)

If `telemetry.factory` is not set, the bridge uses `NullTelemetry`.

## Failure behavior

Factory loading/validation errors fail node startup. Common cases:

- Symbol path does not import.
- Symbol resolves to a non-callable object.
- Factory result does not expose required methods (`authorize` or `emit`).

## Example configuration

```yaml
livekit_bridge:
  ros__parameters:
    access.factory: "my_bridge_ext.access:create_access_policy"
    telemetry.factory: "my_bridge_ext.telemetry:create_telemetry"
```
