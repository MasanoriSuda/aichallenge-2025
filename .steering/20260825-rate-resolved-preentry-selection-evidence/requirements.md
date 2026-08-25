# Requirements

## Objective

Collect the missing evidence needed before atomically promoting the prospective
six-state Overtake Gate A and deleting the tactical five-state owner.

## Requirements

- Preserve the immutable six-state execution artifact and exact static-wall
  certificate produced by each prospective left/right solve.
- Derive branch-comparison metrics from that same six-state solve.
- Compare the six-state left/right decision with the current five-state Gate A
  decision without changing Mission selection or normal command authority.
- Keep the observation path incapable of publishing a command, replacing a
  Mission, or writing the production certified-plan store.
- Do not add a feature flag, fallback, timeout, lease, or parameter tuning.
- Preserve ROS 2 and evaluation interfaces.

## Non-goals

- No production Gate A promotion in this Slice.
- No five-state deletion until dynamic selection and artifact evidence exists.
- No controller parameter changes.

## Definition of Done

- Failure-first source contracts prove observation-only authority.
- Both prospective sides expose a validated immutable six-state CertifiedPlan
  only when solver, wall and target proofs all pass.
- A separate six-state selection result is logged with agreement/disagreement
  against production five-state selection.
- Build and package tests pass.
- A bounded `make dev2` run provides dynamic evidence.
