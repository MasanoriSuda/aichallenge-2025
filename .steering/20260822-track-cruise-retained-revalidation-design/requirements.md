# Track/Cruise retained revalidation requirements

## Purpose

Define the current-observation physical proof required before a retained canonical
five-state Track/Cruise plan may own normal control.  This steering is an audit and
design slice only; it deliberately does not connect retained authority or change a
controller parameter.

## Evidence boundary

- Branch: `develop_july`
- Baseline: `52265834096555bfa258679a4b135d86bf770ef4`
- Evidence: source/history audit and deterministic test design
- Preserved user change: `aichallenge/result-summary.json`
- Dynamic Gate A: accepted for the fresh certified chain using `output/20260822-232351`;
  retained shadow is permitted, retained publisher authority is not

## Observed gap

The canonical plan already preserves exact stage durations, complete Frenet states,
absolute solved progress and three-input controls.  The cursor also advances by elapsed
time without clamping or repeating the final stage.

The retained revalidation contract, however, contains only:

- current decision ID;
- old plan ID;
- remaining stage-index window;
- four physical-certificate values.

It cannot prove which current state observation, course geometry, obstacle observation,
control pose or time/progress sampling window produced those values.  Reusing the old
certificate or indexing current stage arrays by the old cursor would therefore certify a
different physical problem.

## Required invariant

A retained candidate may be constructed only from one immutable proof which binds all of:

1. the current control decision and current Track/Cruise intent;
2. the current ego observation generation;
3. the current target/dynamic-obstacle observation generation, when a target exists;
4. the retained plan ID and exact cursor, including partial first-stage elapsed time;
5. the current course-frame geometry provenance used to interpret absolute progress;
6. the current delay prefix from measured pose to predicted command-application pose;
7. the swept remaining plan beginning at that predicted command-application pose;
8. current wall/corridor constraints sampled by absolute course progress;
9. current obstacle occupancy sampled by elapsed time from the present observation and
   compared at the corresponding absolute progress;
10. the resulting wall and obstacle physical certificate.

If any identity, sample, geometry coverage or physical check is unavailable, retained
normal authority must be unavailable.  The existing selector then reaches Emergency Stop;
it must never fall back to legacy MPC.

## Coordinate and timing rules

- Retained control selection uses elapsed **time**.
- Static course frame and wall/corridor geometry use unwrapped absolute **progress**.
- Dynamic obstacle occupancy uses current-observation-relative **time**, with course
  progress/lateral position as spatial coordinates.
- These axes must not be replaced by a shared stage index.
- On a circular course, current progress must be lifted to the retained plan's unwrapped
  lap branch explicitly.  Silent modulo, extrapolation or nearest-stage substitution is
  forbidden.
- If the cursor lies part-way through stage `k`, the first new obstacle sample and sweep
  segment cover only `duration[k] - stage_elapsed_sec` and terminate at predicted state
  `k + 1`.  Predicted state `k` is historical, not the current pose.

## Non-scope

- No Track/Cruise final-publisher promotion.
- No retained execution in production or shadow runtime.
- No clearance, horizon, solver, cost or speed tuning.
- No new fallback, grace period, retry loop, mode flag or controller branch.
- No Follow/Hold/Stop or overtake authority integration.

## Acceptance for the later implementation slice

- Every failure-first case in `design.md` rejects before the implementation.
- A pure revalidator produces one sealed proof, or an explicit rejection reason.
- Candidate construction accepts only the exact plan/cursor/proof identity.
- Missing current geometry, pose, observation or dynamic tube fails closed.
- Focused tests, full package tests and `make autoware-build` pass.
- Runtime remains disconnected until fresh dynamic Gate A is accepted.
