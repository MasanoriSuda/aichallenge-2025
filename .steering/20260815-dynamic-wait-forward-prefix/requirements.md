# Requirements

## Background

The `20260815-071257` `make dev2` run confirmed that the early-Pass physical
gate and cumulative SafeSeparation lifecycle reduced immediate wall failures.
However, four of seven overtake Missions then remained in
`DynamicMissionWait` until the 15 second total Mission budget expired.  During
those waits both fresh branches were often `planning_unavailable`, the lateral
reference fell back to the base trajectory, and closing authority was reduced
to the generic unlatched value.

## Required behavior

- While `DynamicMissionWait` searches for a replacement, publish a freshly
  wall-validated current-side forward prefix instead of silently returning to
  the base trajectory.
- Preserve useful overtake closing authority only when the current and
  predicted physical footprints are separated; otherwise retain the existing
  bounded unlatched closing policy.
- Continue evaluating both sides and atomically commit a fully admitted
  alternate as soon as it is available before no-return.
- Keep the existing total Mission time budget as the absolute terminal bound.

## Constraints

- Current wall contact/margin, emergency-front-risk, solver, forbidden
  waypoint, target-continuity and body-overlap faults remain fail closed.
- A retained prefix must be rebuilt and wall-validated from the current pose;
  an old target-relative Mission path is not replayed blindly.
- Predicted target overlap must not receive a speed floor.
- Do not change ROS topics, messages, services, launch entry points or result
  schemas.
- Preserve all existing user changes and the preceding uncommitted steering
  implementation.

## Definition of done

- A deterministic core policy distinguishes a safe full-speed forward hold
  from a prediction-conflict limited hold and a hard rejection.
- `DynamicMissionWait` emits an active lateral reference when the current-side
  prefix passes fresh wall validation.
- A fresh alternate retains priority over the hold path.
- Local/cloud configuration remains synchronized.
- Package build, tests and diff checks pass.
