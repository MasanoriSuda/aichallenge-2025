# Requirements

## Background

The `20260815-003734` run confirmed that the bounded target-conflict Pass
prefix can retain forward speed, but no attempt completed `Pass -> Return`.
The cumulative hold was revoked six times by cycles that produced neither a
fresh optimizer horizon nor an explicit non-target hard failure.  In addition,
the active Frenet-DP execution path was refreshed only once; most Pass cycles
kept a several-second-old path until wall or lateral-acceleration feasibility
was lost.

## Goal

Keep target-bound lifecycle accounting across neutral planner gaps and refresh
the active same-side Pass DP reference from the newest physically validated
receding prefix, without waiting for a complete rear-clear/Return Mission.

## Constraints

- Keep actual wall contact, wall-margin, emergency-front-risk, solver recovery,
  target discontinuity, forbidden-waypoint and non-recoverable body overlap
  fail-closed.
- A neutral planner gap may retain lifecycle bookkeeping, but must not execute
  the target-bound physical prefix unless the current failure is target-bound.
- DP refresh must remain exact-target, same-side, prediction-fresh and atomic.
- A rejected refresh must retain the last feasible reference.
- Do not change ROS topics, messages, services, launch entry points or result
  schemas.
- Keep local and cloud parameter files synchronized.

## Definition of Done

- The target-bound lifecycle distinguishes target conflict, fresh solution,
  neutral gap and explicit hard failure.
- A neutral gap does not release or rearm the cumulative time/distance budget.
- Behavior output exposes a current-side receding DP-prefix candidate separately
  from the complete Mission replacement candidate.
- Pass DP execution can refresh from that prefix under the existing target,
  side, freshness and interval gates.
- Unit tests cover lifecycle neutral gaps, hard revocation and DP-prefix refresh
  admission.
- The package builds and focused tests pass.
