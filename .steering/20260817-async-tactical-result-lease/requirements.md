# Requirements

## Purpose

Keep a validated V2X overtake candidate and execution prefix available while
the asynchronous tactical worker computes its next result.  A temporary
worker gap must not make a predictive overtake activation fall back to Follow,
nor erase the candidate before a due replan cycle can consume it.

## Scope

- Lease the latest accepted asynchronous tactical result for the same target,
  Mission generation, phase, side and worker context.
- Reuse the leased result only for a bounded age and revoke it on every current
  hard fault.
- Size the receding-horizon execution lease from the asynchronous evaluation
  cadence, while keeping the configured tactical-result maximum age as its
  upper bound.
- Preserve all wall, target continuity, forbidden-waypoint, emergency-front
  risk and solver-recovery fail-closed guards.

## Constraints

- Do not modify the user's `config.yaml` change or generated result files.
- Do not change ROS 2 topics, services, message types or launch contracts.
- Do not weaken physical collision or wall-contact guards.
