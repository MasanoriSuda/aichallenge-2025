# Validation

## Static verification

- `test_mpc_stage_geometry`: 8 passed.
- `test_recovery_footprint`: 57 passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 1,732 tests,
  zero failures/errors/skips. `colcon test-result` also reported the known
  unrelated stale `joycon_contract_guard/package.xml` lookup.
- `make autoware-build`: 25 packages succeeded.
- `git diff --check`: clean.

The tests prove that a rejected swept segment reports the actual interpolated
collision pose, and that course/Frenet resampling follows a curved reference
instead of the sparse world-frame chord.

## Dynamic verification

`make dev2` produced `output/20260824-065336`. The run did not produce a
world-chord rejection with `course_sweep=...`, so it cannot authorize a wall
proof behavior change. It did produce a stronger upstream failure-first case:
ShiftOut was admitted with a tactical exact-stage certificate while the
canonical Overtake worker was still pending, after which live proof rejected
the missing execution trajectory and rolled entry back.

## Gate result

Accept the observability changes only. Do not replace the fail-closed sparse
world sweep yet. The next Slice must repair atomic Overtake authority promotion
and obtain an exact canonical execution artifact before `Idle -> ShiftOut`.
After that, repeat this comparison to decide whether the continuous-wall defect
is a validator interpolation artifact or a missing QP constraint.
