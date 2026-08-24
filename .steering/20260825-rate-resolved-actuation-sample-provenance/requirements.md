# Requirements

## Objective

Explain every `ActuationSampleRejected` result observed in
`output/20260825-004100` before changing any timing, steering bound, solver
tolerance or production authority.

## Evidence

The six-state Track/Cruise runtime shadow built and solved all 4,354 consumed
QPs, but only 3,859 supplied a certified 25 ms actuator sample. The existing
optional-return validator collapses publication-time, steering-angle,
steering-rate and numerical-boundary failures into one string.

## Scope

- Replace the opaque sample validation path with a typed reason contract.
- Preserve the existing optional API as a compatibility wrapper.
- Carry the exact reason and relevant physical values into shadow telemetry.
- Add deterministic reason-boundary tests.
- Replay `make dev2` and identify the earliest violated invariant.

## Non-scope

- No tolerance, bound, horizon, solver or configuration change.
- No value clamp or fallback.
- No production authority or command change.
- No physical-certificate or warm-start promotion.

## Preserved user state

`aichallenge/result-summary.json` is a pre-existing user change and must not be
edited, staged or committed.

## Rollback

Rollback target: `54023ee`.
