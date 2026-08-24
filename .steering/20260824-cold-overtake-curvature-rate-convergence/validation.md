# Validation

## Build

- `make autoware-build`: passed after the producer repair.
- Targeted package rebuild after the final telemetry clarification:
  `colcon build --symlink-install --packages-select multi_purpose_mpc_ros ...`:
  passed.

## Static tests

- Focused CTest:
  `test_mpcc_progress`, `test_persistent_osqp`, and
  `test_race_mpcc_foundation`: 3/3 passed.
- Full `multi_purpose_mpc_ros` CTest: 40/40 passed.
- `git diff --check`: passed.

The new unit tests cover exact steering-coordinate transformation, input-box
intersection, empty intersection reporting, and malformed request rejection.

## Dynamic regression

- Command: bounded `make dev2`, followed by `make down`.
- Run: `output/20260824-180631/`.
- Domain 1 log duration: approximately 72.5 seconds.
- Before repair:
  - `output/20260824-172712`: row 270 was 25/25 failed rows.
  - `output/20260824-174555`: row 270 was 82/88 failed rows.
  - `output/20260824-175458`: row 270 was 18/20 failed rows.
- After repair:
  - row 270 was 0/2 failed rows.
  - remaining rows were 271 once and 211 once.
  - no empty first-curvature intersection was reported.

## Interpretation

The stage-zero actuator limit is unchanged physically. Its exact reachable
interval now narrows the canonical stage-zero curvature box, while the old
duplicate rate row is structurally retained but unbounded. This preserves the
dual row layout and removes only the duplicate numerical owner.

The remaining row-271 and row-211 failures are not suppressed. They are a
separate convergence investigation boundary, not grounds for changing OSQP or
vehicle parameters in this Slice.
