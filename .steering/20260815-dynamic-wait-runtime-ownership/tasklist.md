# Task list

- [x] Add an explicit DynamicMissionWait runtime ownership predicate.
- [x] Connect the controller branch through the predicate.
- [x] Add truth-table unit coverage for ownership admission.
- [x] Run formatting/diff checks.
- [x] Build the Autoware workspace.
- [x] Run `multi_purpose_mpc_ros` tests.

## Verification

- `git diff --check`: passed.
- `make autoware-build`: 25 packages completed successfully.
- `multi_purpose_mpc_ros`: 25/25 test targets passed.
- `colcon test-result --verbose`: 1143 tests, 0 errors, 0 failures,
  0 skipped.  The existing missing `build/joycon_contract_guard/package.xml`
  diagnostic remains unrelated to this package's passing result.

## Definition of done

- Rolling replan plus active `DynamicMissionWait` reaches the executor.
- Rolling replan without an active wait preserves previous behavior.
- Existing replacement priority and hard guards remain unchanged.
- Build and package tests pass.
