# Requirements

## Goal

Replace the inherited hard-curve abort preference with an explicit simulation-race pass-side
policy: choose a sufficiently open inside corridor, otherwise attack around the outside.

## Functional requirements

- Do not reject a new or active pass only because a hard curve was detected when a geometrically
  valid inner or outer corridor is executable.
- Before a pass side is locked, prefer the inside only when that corridor remains continuously
  open for a configurable path distance. If it does not, prefer a feasible outside corridor.
- If neither condition is satisfied, remain in Follow/SafetyBrake according to the existing
  front-risk arbitration.
- After ShiftOut starts, retain the locked target and pass side through Pass and Return. Do not
  switch from inside to outside, or vice versa, during the maneuver.
- Keep explicit forbidden waypoints, cooldown, EmergencyBrake, target continuity/intrusion,
  inflated-vehicle gap, wall clearance, solver, and odometry guards.
- Keep ROS topics, messages, services, Domain layout, and evaluation result contracts unchanged.

## Acceptance

- Unit tests cover inside selection, outside fallback, insufficient inside-only clearance, and
  locked-side retention.
- The current dev3 profile requires at least 3.0 m of continuous usable inside corridor.
- `make autoware-build` and the `multi_purpose_mpc_ros` tests succeed.
- Runtime confirmation checks that hard-curve detection does not directly cause
  `Overtake -> Follow` while one of the selected-side corridors remains valid.
