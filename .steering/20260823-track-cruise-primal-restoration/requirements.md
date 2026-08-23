# Track/Cruise feasible-primal restoration (rejected)

## Objective

Replace the rejected-solver-primal execution boundary with one canonical,
dynamically consistent execution candidate. The candidate must be generated
from the same measured initial state, the same five-state linearizations and
the nearest controls inside the already-declared QP box/rate constraints.

This Slice addresses the root defect demonstrated by
`output/20260823-075629`, `output/20260823-081219` and
`output/20260823-084006`: OSQP's globally accepted solution can miss a
small-unit actuator/state row, while independently clamping individual fields
would destroy the dynamics equality used to certify the horizon.

## Constraints

- Do not widen a bound, tolerance, wall margin or velocity limit.
- Do not change objective weights or OSQP settings.
- Do not add a retry, timeout, feature flag or legacy authority fallback.
- Preserve the raw OSQP primal/dual exclusively for solver telemetry and warm
  start provenance.
- Treat restoration as a new candidate; it receives authority only after all
  QP rows and the existing physical/world certificates pass again.
- Project curvature sequentially through both its box and curvature-rate
  intervals. Never clamp each curvature stage independently.
- Roll every state from the measured stage-zero equality through the exact
  linearized dynamics used to build the QP.
- Fail closed on an empty rate/box intersection, a rolled state outside its
  box, a malformed matrix/bound contract or any final QP-row violation.
- Preserve ROS, launch, topic and publisher contracts.

## Acceptance

- Failure-first tests reproduce curvature, acceleration, virtual-progress and
  predicted-velocity boundary misses.
- Restored candidates satisfy every QP row under a strict local tolerance.
- A candidate whose bounded controls roll outside a state/corridor bound is
  rejected rather than hidden.
- Raw OSQP output remains the warm-start source.
- Complete package tests and build pass.
- One-car dynamic run materially reduces `execution-primal-reject` without a
  new wall/recovery incident or callback overrun.

## Decision

Rejected on dynamic evidence from `output/20260823-090945`. Exact state
rollout changed 120--145 primal fields per sampled cycle and required
0.58--3.13 maximum adjustment. This is not a small actuator-boundary repair;
it exposes a materially inconsistent raw horizon. The experiment produced
restoration rejects, physical wall rejects, Emergency authority and Reverse.
All behavior and test code from this Slice was removed. Only this audit record
is retained.
