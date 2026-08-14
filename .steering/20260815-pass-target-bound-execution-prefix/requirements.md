# Requirements

## Background

The `20260815-001418` run reached `ShiftOut -> Pass`, but no pass completed.
During the best attempt the live optimizer alternated between a feasible Pass
horizon and `optimized horizon escaped target separation bounds`. The existing
physical-prefix hold was limited by the generic 0.30 s observation continuity
lease, and a single fresh horizon immediately cleared its accumulated state.
This produced repeated hold start/resolve chatter followed by FollowPrepare.

## Goal

Keep executing a physically feasible current-side Pass prefix while a
target-only optimizer conflict is replanned, without requiring a complete new
Mission path on every control cycle.

## Constraints

- Apply only in `Pass` with a frozen Mission and a target-bound-only failure.
- Keep wall contact, wall-margin, emergency-front-risk, solver-recovery,
  forbidden-waypoint, target-jump, and non-recoverable body-overlap checks hard.
- Do not change ROS topics, messages, services, launch entry points, or result
  schemas.
- Do not weaken the hard target bounds globally.
- Bound the forward continuation by both elapsed time and traveled distance.
- Do not reset the cumulative budget on a one-cycle feasible optimizer result.

## Definition of Done

- Target-bound Pass prefix uses its own configurable time/distance budget.
- A fresh optimizer result must remain continuous for a configured stabilization
  time before the cumulative prefix state is released.
- A renewed target-bound conflict before stabilization resumes the original
  cumulative budget rather than rearming it.
- Unit tests cover start, stable release, renewed conflict, budget exhaustion,
  and hard-fault rejection.
- Local/cloud configurations remain identical for the new parameters.

