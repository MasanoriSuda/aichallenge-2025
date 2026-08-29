# Requirements

## Objective

Eliminate the semantic split observed in `output/20260829-155509`: a
certified ShiftOut trajectory lost normal authority, longitudinal control
became an emergency stop, but lateral control continued the previous ShiftOut
steering and drove the vehicle into the wall.

## Constraints

- Do not add a Mission resume rule, lease, grace period, timeout, fallback
  branch, solver tolerance change, or clearance change.
- Keep the seven-state canonical normal producer unchanged.
- Keep Stop outside normal MPCC authority.
- Preserve target, homotopy and Mission state; this Slice changes only the
  authority identity and lateral action of an already-required emergency stop.
- A moving emergency stop may not hold an arbitrary previous steering angle.

## Acceptance

- Every canonical normal authority failure publishes as the existing external
  `Stop` authority.
- Moving Stop uses the existing rate-limited reference-path steering policy;
  zero-speed Stop may hold its steering.
- The published authority identity and final decision trace both report Stop.
- Existing normal Track/Cruise/Follow/ShiftOut/Pass/Return/Rejoin production
  authority is unchanged when a certified artifact exists.
