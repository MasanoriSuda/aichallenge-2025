# Requirements

## Objective

Attribute every 25 ms production control-callback overrun to a bounded runtime
region before changing cadence, solver settings, certificates or behavior.

## Evidence

- `output/20260825-020710`: one 25.644 ms overrun
- `output/20260825-021144`: one 27.513 ms overrun
- The asynchronous rate-resolved shadow had no failure and stayed below
  11.645 ms, so it is not proven to own either production overrun.

## Scope

- Measure pre-MPCC preparation, `MPC::get_control()`, post-MPCC execution/wall
  arbitration, Stuck Recovery evaluation, and publish/trace work.
- Emit one exact decision-correlated warning only when the callback exceeds its
  immutable period.
- Preserve the existing one-second aggregate.
- Keep all measurements observation-only.

## Non-scope

- No control-rate, solver, certificate, wall, horizon or log-cadence tuning.
- No work offloading or new fallback before the owner is identified.
- No authority or command changes.

## Preserved user state

`aichallenge/result-summary.json` is user-owned and excluded.

## Rollback

Rollback target: `a9938e3`.
