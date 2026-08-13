# Tasklist

- [x] Trace Mission path generation and MPC target injection.
- [x] Define bounded receding-horizon planner and fallback boundary.
- [x] Add pure continuous lateral optimizer and unit tests.
- [x] Integrate live wall/target/course samples into ShiftOut and Pass.
- [x] Add YAML parameters to local and cloud configurations.
- [x] Run focused tests and `make autoware-build`.
- [x] Record verification results and remaining dynamic-test items.

## Verification

- `make autoware-build`: successful (25 packages).
- `colcon test --packages-select multi_purpose_mpc_ros`: 25 test targets passed.
- `colcon test-result --all --verbose`: 1069 tests, 0 errors, 0 failures.
- `git diff --check`: clean.

## Dynamic test checklist

- Run `make dev2` with the normal fast/slow pair.
- Confirm periodic debug contains `rh=1/fallback=0` during ShiftOut/Pass.
- Compare `fallback=1` count, wall Recovery count, Pass minimum speed, and
  ShiftOut-to-rear-clear time against the preceding HEAD.
- Inspect whether the target lateral sequence bends toward the curve outside
  without crossing the selected target side during body overlap.
