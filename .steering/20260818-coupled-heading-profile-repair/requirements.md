# Requirements

## Purpose

Recover the overtake completion regression introduced by point-wise
heading-aware wall repair while retaining the physically correct vehicle-yaw
validation.

## Evidence

For the first 332.7 seconds of D1:

| Event | `20260818-003303` | `20260818-064806` |
|---|---:|---:|
| Optimized horizon physical-revalidation failures | 1 | 14 |
| Solved MPCC hard-wall releases | 7 | 21 |
| Static wall-margin failures | 0 | 5 |
| `Pass -> Return` | 0 | 0 |

The new run entered Pass 11 times, but never completed Return. There were no
MPC solver failures and V2X remained healthy, so the immediate regression is
inside lateral-profile wall revalidation.

## Scope

- Participant package only: `aichallenge_submit/multi_purpose_mpc_ros`.
- Preserve the heading-aware footprint check, physical kart footprint and hard
  wall margin.
- Preserve all ROS 2 topics, services, launch files and parameter schema.
- Do not include the user's local `config.yaml` or result JSON changes.

## Acceptance criteria

- Static-wall validation uses the same backward `current -> stage` heading
  convention as the executed `target_epsi` profile.
- Wall repair contracts a complete lateral profile toward a smooth fallback;
  it does not independently move one stage and increase adjacent `d'(s)`.
- A future candidate failure does not become physical Recovery while a
  revalidated current/last-feasible prefix remains available.
- Unit tests, package tests and the Autoware build pass.
