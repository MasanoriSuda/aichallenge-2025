# Requirements

## Purpose

Prevent rapidly changing opponent bounds from destabilizing the tracking MPC
while preserving the Mission-wide hard wall corridor introduced in `4c9a22d`.

## Evidence

The `20260817-065615` run confirmed that wall bounds stayed active through
target-bound holds and Return, but exposed a second-order failure:

- target-bound tracking constraints repeatedly alternated between enabled and
  wall-only at the 40 Hz control boundary,
- one Pass reached more than 3000 average OSQP iterations before
  `maximum iterations reached`,
- a later Pass entered Recovery and accumulated hundreds of failed solve
  cycles before a long stuck-recovery incident,
- only two of fourteen initiated overtake episodes completed Return -> Idle.

## Required behaviour

- Wall bounds remain hard for every physically evaluated active Mission
  horizon.
- Opponent bounds are promoted into the tracking MPC only after a continuous
  confirmation interval.
- Any candidate dropout immediately falls back to current wall-only bounds;
  stale opponent geometry is never retained as a hard constraint.
- A solver failure suppresses opponent-bound promotion for a bounded cooldown
  while wall-only MPC receives the next solve opportunity.
- After cooldown, opponent bounds must satisfy the full confirmation interval
  again before promotion.
- Existing Mission planning, contact policy, Recovery and ROS interfaces are
  unchanged.

## Acceptance criteria

- Wall-only bounds and wall-plus-target bounds are exported separately.
- The promotion gate is deterministic and unit tested for confirmation,
  dropout and solver cooldown.
- Runtime logging distinguishes candidate target bounds, effective target
  bounds, confirmation wait and solver suppression.
- Local/cloud parameters remain synchronized.
- Build and package tests pass.
