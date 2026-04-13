# Logging Guide

Use these logging rules consistently in `livekit_ros2_bridge`.

The bridge uses flat structured logs built with `LogEvent`: each line starts with `event=...` and
then adds `key=value` pairs.

## Core Rules

- Keep `event` names stable and descriptive. Prefer improving one existing event over adding a near-duplicate event.
- Keep fields minimal. Include the fields that answer what happened, to what, and why.
- Do not carry extra context through multiple layers just to log a low-value field. If the field is not decisive, drop it.
- Prefer one strong log with a stable field set over several nearby logs that each carry slightly different context.

## `field` vs `fieldOr`

- Use `field(...)` for required fields that should always be present.
- Use `fieldIf(...)` when a simple boolean guard should keep the log builder chainable.
- Use `fieldIfNotEmpty(...)` for optional string, `string_view`, or C-string fields when omission is better than a placeholder.
- Use `fieldOr(...)` for optional string or string-like fields when keeping the key present improves comparison across related events or makes a shared log helper simpler.
- Prefer `fieldOr(...)` over repeated hand-written `if (!value.empty()) event.field(...)` checks when the fallback value is acceptable and the stable key helps readability.
- Prefer `fieldIf(...)` or `fieldIfNotEmpty(...)` over temporary `LogEvent` variables when the only goal is to gate one or two fields inline.
- Do not use `fieldOr(...)` just to add noisy placeholders for fields that are not useful on most events.

## Optional And Count-Like Fields

- For numeric fields that are useful only when non-zero, prefer omitting them at zero.
- When several events or one summary log need the same non-zero-only behavior, use a tiny local helper such as `addCountFieldIfNonZero(...)` instead of repeating the conditional at every call site.
- Keep such helpers local to the file unless there is clear reuse across multiple files.

## Shared Field Builders

- When multiple logs in one file share the same identity or context fields, prefer a small local helper that appends those fields to a `LogEvent`.
- Good shared fields include stable identifiers like `resource`, `service`, `requester_identity`, `stream_key`, `track_name`, `interface_type`, `operation`, `reason`, and `stage`.
- Do not extract a helper if the shared field set is too small or if the helper would hide the event-specific meaning.

## What To Remove

- Remove fields that only restate the event name, logger, nearby method name, obvious local type, or temporary representation details.
- Remove logs that only narrate straightforward control flow.
- Remove fields whose only purpose is to justify carrying extra context that the code otherwise does not need.
