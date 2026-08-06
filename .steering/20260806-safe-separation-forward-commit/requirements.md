# Requirements

## Problem

The submitted `20260805-150308` run reached Pass repeatedly but completed it
only once.  Its Pass exits were 17 Recovery transitions and one Return.  Ten of
the Recovery transitions were:

```text
SafeSeparation target clear ahead confirmed; recovering behind
```

The current SafeSeparation policy deliberately commands below the target speed
when the target is ahead, even when the committed minimum-motion corridor is
still valid and both vehicle bodies are separated.  This converts a recoverable
Pass into Follow-like backing off.

## Required behavior

- A committed, body-separated, continuity-valid Pass may continue forward
  escape while the target remains inside a bounded front window.
- Confirming that the target is ahead must not override an authorized forward
  escape inside that window.
- The forward reference must never reduce the current ego speed and should use
  the configured closing-speed delta.
- Rear-clear still transitions to Return immediately.
- Unsafe short horizon, timeout or distance exhaustion still aborts to the
  existing Recovery path.
- Outside the authorized forward window, the existing recover-behind behavior
  remains available.

## Constraints

- Keep wall, body-overlap, target-continuity, emergency and solver guards.
- Do not change ROS interfaces or evaluation schemas.
- Keep `a_max=1.0 m/s^2` and the global/domain speed limits unchanged.
- Preserve all existing user changes, including the V2X peer auto-discovery
  work and `aichallenge/result-summary.json`.

## Definition of Done

- Forward escape takes precedence over target-ahead recovery only when the
  caller's existing strict authorization is true.
- The configured window and bounded duration cover the reproduced 2 m
  target-ahead transition.
- Focused tests cover precedence, out-of-window fallback and hard bounds.
- The package builds and focused tests pass.

