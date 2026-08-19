# Requirements

## Goal

Prevent a committed Pass from being discarded when the target classification
briefly drops out while a recently solved, dynamically validated MPCC prefix is
still authoritative.

## Observed failure

In `output/20260819-115628`, episode 4 entered Pass through the new physical
admission gate and continued with clear target footprints and a feasible
receding horizon. At waypoint 175, the live target classification was lost
briefly. The Pass horizon refresh reported `fresh target prediction
unavailable` and transitioned to Recovery even though:

- the V2X source still had approximately 0.95 s of TTL;
- the committed dynamic prefix remained valid from 5.64 m to 9.19 m; and
- the lower receding-horizon controller had a bounded last-feasible execution
  lease.

The upper Pass lifecycle therefore revoked authority before the lower MPCC
continuity policy could use its already validated solution.

## Constraints

- Retain execution only for a recent target observation and recent clear
  footprint prediction.
- Require a still-valid committed dynamic prefix and a live cached MPCC
  execution lease.
- Physical wall contact, wall-margin failure, missing wall samples, confirmed
  overlap, pass-side intrusion, emergency braking, forbidden waypoints, course
  discontinuity, and solver Recovery remain immediate revocation conditions.
- Do not extend immutable Pass time or distance budgets.
- Do not change ROS interfaces, wall margins, acceleration limits, MPCC
  weights, or launch/config schemas.
- Preserve the user's `aichallenge/result-summary.json` modification.

## Definition of Done

- Dynamic-horizon rejection reasons distinguish target dropout from corridor,
  overlap, and course-continuity failures.
- A transient target-only dropout can retain the current cached MPCC execution
  for the existing bounded continuity lease.
- The lease cannot admit a stale, physically unsafe, or budget-exhausted path.
- Focused unit tests and the package build pass.
