# Requirements: Track/Cruise Slice 3 closure

## Objective

Decide whether the remaining five-state OSQP numerical unavailability is a
Track/Cruise authority-migration blocker or a separately bounded solver-quality
risk.  Close Slice 3 only with an uninterrupted six-lap production run from a
runtime artifact rebuilt from the current source tree.

## Constraints

- Do not change solver settings, tolerances, weights, bounds or racing parameters.
- Do not add a fallback, retry, clamp, feature flag or alternate normal authority.
- Do not weaken the semantic or physical execution certificates.
- Preserve the sole normal Track/Cruise chain:
  `fresh canonical -> current-world retained canonical -> Emergency Stop`.
- Exclude a run if the installed executable contains source that is absent from
  the current Git tree.
- Do not edit or stage `aichallenge/result-summary.json` or generated outputs.

## Acceptance

- One uninterrupted six-lap run completes.
- Callback overrun, wall/contact event, abrupt external speed loss, confirmed
  Stuck, Reverse and Recovery are zero.
- No Track/Cruise legacy/three-state normal command is published.
- Every unavailable fresh solution fails closed through the typed canonical
  Emergency path; it is not relabelled or repaired downstream.
- Remaining numerical unavailability is quantified and assigned to a future
  solver/formulation Slice rather than hidden by another patch.
