# Task list

- [x] Add a tested live/worker tactical ownership decision.
- [x] Make complete side assessments transferable in async results.
- [x] Consume and validate the result before live side comparison.
- [x] Defer expensive live candidate generation to the worker.
- [x] Update startup diagnostics.
- [x] Run package build and tests.
- [x] Commit only scoped source/config/steering changes.

## Definition of Done

- Async mode has one tactical candidate generator: the worker.
- Live hard guards and frozen Mission continuity remain active.
- No interface contract changes.
- Build/tests pass and the next `make dev2` run can compare callback overruns,
  worker adoption and OvertakeLine transitions.

## Verification

- `docker compose run -T --rm --no-deps autoware-build`: passed (25 packages).
- `colcon test --packages-select multi_purpose_mpc_ros`: 1223 tests, 0 errors,
  0 failures, 0 skipped.
- Final targeted `ctest -R test_latest_only_worker`: passed (1/1).
