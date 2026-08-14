# Requirements

## Background

The `20260815-012736` run confirmed that receding-prefix DP refresh is active:
22 of 24 refresh logs used `source=receding_prefix`, and the first encounter
completed `Idle -> ShiftOut -> Pass -> Return -> Idle` in about 6.65 seconds.
The following encounter still repeated `same-side lateral adjustment limit
exceeded`, SafeSeparation and Recovery.

Two authority handoff defects remain:

1. a fresh, same-target/same-side DP prefix is accepted for execution, but the
   legacy single-goal Pass extension can reject the same cycle;
2. Return preflight validates a live return horizon, discards it, and the next
   Return cycle regenerates a different legacy horizon.

## Goal

Keep a recently refreshed, physically admissible DP prefix authoritative for
bounded receding-horizon Pass continuation, and execute the exact horizon that
was admitted by Return preflight.

## Constraints

- DP authority is retained only for the same target and same side while target
  continuity and the configured last-feasible age hold.
- Actual wall contact/margin failure, unavailable wall samples, emergency front
  risk, target discontinuity, forbidden waypoints and solver Recovery remain
  hard revocation conditions.
- Legacy complete-Mission extension remains the fallback when DP authority is
  absent or stale.
- Return preflight does not weaken wall or lateral-acceleration validation.
- Do not change ROS topics, messages, services, launch entry points or result
  schemas.
- Preserve the user's existing `aichallenge/result-summary.json` change.

## Definition of Done

- Pure policy tests cover DP authority acquisition and every hard revocation.
- A RequestSameSideExtension/RequestLongitudinalRefresh does not invoke the
  legacy single-goal rebuild while fresh DP authority is active.
- Return phase consumes the preflight-admitted lateral reference.
- Exhausted or invalid Return references fall back to the existing Return
  profile.
- Focused tests, full package tests and package build pass.
