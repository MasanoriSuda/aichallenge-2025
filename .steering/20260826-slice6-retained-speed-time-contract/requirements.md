# Requirements

## Objective

Eliminate the first causal six-state authority loss observed after a valid
ShiftOut admission.  The exact trajectory accepted by Gate A must become the
initial active execution prefix; an older tactical DP path must not replace it
between admission and the first production solve.

## Constraints

- Keep one `VelocitySteeringProgress6State` normal authority.
- Do not change OSQP parameters, wall margins or tactical thresholds.
- Do not add a fallback, lease, timeout or exception path.
- Preserve fail-closed behavior for a genuinely unreachable retained command.
- Use source timestamp evidence; do not hide model divergence with an arbitrary
  velocity tolerance.
- Gate A Mission geometry and its exact six-state physical proof must cross the
  FSM boundary atomically.
- Do not stage generated result JSON.

## Exit criteria

- Rosbag evidence has falsified a source-timestamp mismatch for this event.
- The committed Mission carries the exact six-state trajectory and wall/target
  provenance accepted by Gate A.
- Failure-first tests prevent a tactical DP path from overwriting that exact
  prefix.
- Build, package tests and moving acceptance pass.
- No retained velocity rejection immediately precedes six-state Emergency in
  the acceptance scenario.
