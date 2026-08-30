# Requirements

## Objective

Eliminate the structural loss of both normal-avoidance branches when their
course-frame window crosses the duplicated waypoint at the circular-path seam.

## Frozen evidence

- Baseline: `0f15a634`
- Run: `output/20260830-110056/d1/autoware.log`
- Sequences 3460 and 3541 reject both sides with
  `dynamic-target-unavailable/target course projection unavailable at stage 0`.

## Constraints

- Do not change production authority, Mission lifecycle, solver settings,
  timing policy, wall clearance, or obstacle clearance.
- Continue to fail closed when either the ego progress bounds or the captured
  target tube is malformed.
- Derive the frame window only from data already sealed in the immutable
  seven-state snapshot.

## Definition of Done

- Geometrically degenerate segments do not invalidate other usable course
  segments.
- A course containing only degenerate geometry still fails closed.
- Full package tests and build pass.
- A dynamic run across waypoint 338 no longer reports the frozen stage-0
  projection rejection.
