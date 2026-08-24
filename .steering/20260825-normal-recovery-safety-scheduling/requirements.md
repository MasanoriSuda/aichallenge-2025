# Requirements

## Objective

Remove synchronous Recovery map work from control cycles that cannot enter
Recovery, without changing Stuck detection, active Recovery safety, MPCC
authority, solver settings or commands.

## Root-cause evidence

- `output/20260825-023027/d2/autoware.log` recorded decision 2096 at
  26.098 ms against a 25 ms period.
- `MPC::get_control()` used 20.786 ms and Recovery evaluation used another
  5.046 ms in ordinary Cruise with no target and no Recovery command.
- `evaluate_stuck_recovery()` currently classifies a wall and samples the
  current footprint before `StuckDetector` rejects a clearly moving vehicle.

## Required behavior

- Every non-Normal Recovery state retains the full current safety evaluation.
- A Normal-state vehicle that is slow enough and has forward intent retains the
  full current safety evaluation and can still begin the observation window.
- A clearly moving Normal-state vehicle must still update/reset the Stuck
  detector and supervisor, but must not execute occupancy-grid Recovery safety
  work that cannot affect that cycle's action.
- Invalid/non-finite eligibility inputs fail conservatively into full safety
  evaluation.
- No threshold or output command changes.

## Non-scope

- No MPCC optimization, cadence, horizon or OSQP changes.
- No Recovery threshold, distance, timeout or maneuver changes.
- No asynchronous Recovery worker.
- No parameter tuning.

## Preserved user state

`aichallenge/result-summary.json` is user-owned and excluded.

## Rollback

Rollback target: `c2f5d2d`.
