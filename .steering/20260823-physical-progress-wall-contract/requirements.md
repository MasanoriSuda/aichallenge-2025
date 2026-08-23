# Requirements

## Objective

Make the five-state MPCC optimize against the lateral corridor at the vehicle's
solved physical progress instead of the nominal fixed horizon index.  The QP
and the existing exact world-space execution certificate must refer to the same
part of the course.

## Confirmed defect

The extended state is `[e_y, e_lag, e_psi, v, theta]` and its dynamics permit
physical forward motion to be represented by `theta + e_lag`.  The current QP
nevertheless applies the lateral bounds copied from each nominal legacy stage
to `e_y` alone.  A solution can therefore remain legal in the QP while its
reconstructed physical pose lies roughly one metre behind that stage and in a
different wall corridor.  The exact execution certificate then rejects it, or
the vehicle reaches wall contact before a fresh authority can be adopted.

## Constraints

- Do not tune wall margins, solver tolerances, weights, speeds or steering.
- Do not add a fallback, feature flag, timeout or special waypoint rule.
- Preserve the five-state coordinate and the exact physical wall certificate.
- Fail closed when the progress/corridor provenance is malformed.
- Keep ROS 2 interfaces and evaluation contracts unchanged.
- Preserve the unrelated user change in `aichallenge/result-summary.json`.

## Definition of Done

- Failure-first tests demonstrate that a state which is legal under the old
  fixed-stage bound is rejected at its solved physical progress.
- Every predicted five-state stage has lower and upper affine corridor rows
  coupled to lateral, lag and progress.
- The old predicted lateral box is no longer a second, conflicting corridor;
  stage zero remains the measured equality contract.
- Solver residual validation and semantic execution normalization understand
  the new constraint schema.
- Package build and all package tests pass.
- A dynamic run has no increase in solver failures, callback overruns or
  physical wall-certificate rejects relative to `output/20260823-081219`.
