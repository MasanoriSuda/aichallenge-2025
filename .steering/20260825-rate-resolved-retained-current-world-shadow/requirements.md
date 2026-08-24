# Requirements

## Objective

Provide a shadow-only current-world admission proof for the retained
steering-rate six-state Track/Cruise MPCC artifact.  This Slice must preserve
the same formulation and must not grant command authority.

## Root-cause constraint

The retained store currently keeps the solved artifact and the physical-wall
result, but drops the immutable wall grid, footprint and course-frame window
which produced that result.  A later caller therefore cannot prove that the
old suffix certificate still refers to the current static world.

## Required invariants

- A certified plan owns the exact immutable physical source evidence used by
  the accepted wall result.
- Source snapshot, result and artifact identities must match exactly.
- Retained admission requires current Track/Cruise intent and a current,
  explicitly empty V2X observation.
- Circular course progress is lifted to the retained branch and must remain
  within the existing continuity certificate.
- The measured-to-control delay path and current-control-to-retained connector
  are footprint swept against the same immutable wall grid.
- The sampled steering and speed must be reachable from current actuation
  state over one publication interval; no clamp may manufacture reachability.
- The original suffix is reusable only while wall-grid identity, footprint and
  course-frame evidence are unchanged.
- The result is telemetry/proof only: no ROS command, publisher or production
  authority representation may exist in this module.
- No parameter tuning, timeout, lease or fallback is added.

## Definition of Done

- Pure deterministic tests cover accepted and every fail-closed boundary.
- Source contract proves the path remains shadow-only.
- Runtime logs distinguish cursor, semantics, obstacle, source-world,
  progress, connector, steering and velocity rejection.
- Build, package tests and a short `make dev2` gate pass without callback
  overrun regression.
