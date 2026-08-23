# Track/Cruise scaled-termination contract

## Objective

Eliminate the known five-state Track/Cruise defect in which OSQP reports
`solved` under a mixed-unit global residual while the executable curvature,
acceleration, velocity, or virtual-progress row still violates its own
physical-unit tolerance.

This Slice is a bounded solver-contract experiment.  It does not change wall
margin, speed, steering, weights, Recovery, authority leases, or legacy MPC.

## Failure evidence

The unattended six-lap run `output/20260823-104739` reproduced:

- 109 `execution-primal-reject` outcomes;
- 80 curvature rejects at stage 0;
- 126 canonical Track/Cruise Emergency decisions;
- 13 maximum-iteration solve failures;
- one contact/Recovery incident and one Reverse maneuver;
- zero callback-overrun log events.

The rejection path is:

```text
OSQP solved
-> per-row executable contract rejects a small-unit row
-> retained proof unavailable with V2X NoData
-> one-cycle Emergency Brake
```

## Constraints

- Configure scaled termination only on the dedicated Track/Cruise five-state
  solver context.
- Keep shared/legacy MPC and tactical left/right solver construction unchanged.
- Preserve raw physical-unit primal, dual, residual, and warm-start contracts
  at the caller boundary.
- Do not weaken per-row execution normalization or any physical certificate.
- Do not add a fallback, retry, timeout, lease, parameter, or runtime flag.
- Remove the candidate completely if it increases maximum-iteration,
  callback-overrun, wall/contact, or Recovery incidence.

## Acceptance

- Failure-first tests prove default and dedicated solver construction select
  different termination contracts without changing the default.
- Complete package build and tests pass.
- A normal single-car dynamic run materially reduces
  `execution-primal-reject` from 109/6 laps.
- No callback overrun, contact, Recovery, or continuous solve-failure cascade.
- The production trace identifies scaled termination on the dedicated
  Track/Cruise outcome.

