# Validation

## Static verification

- `make autoware-build`: passed, 25 packages.
- Focused CTest selection: 6 targets passed.
- Focused assertions include:
  - equal occupancy-grid deep copies have equal fingerprints;
  - changed cell or geometry changes the fingerprint;
  - retained current-world proof accepts an identical deep copy;
  - retained current-world proof rejects a changed deep copy;
  - adoption shadow remains disconnected from Mission and publication.
- `test_single_authority_source_contract.py`: 52 passed.
- Full package suite: 49 CTest targets, 1870 tests, zero errors and
  zero failures.
- `colcon test-result --verbose` also reports the pre-existing stale
  `build/joycon_contract_guard/package.xml` artifact warning.
- `git diff --check`: passed before full-suite verification.

## Dynamic verification

Baseline evidence:

- `output/20260825-194808`: selected six-state plan was rejected before the
  dynamic join with `static-world-mismatch` solely because the worker owned an
  equivalent deep-copied wall grid.

Post-fix bounded `make dev2` evidence:

- `output/20260825-200059`, domain 1.
- Valid six-state ShiftOut selections reached the live adoption shadow.
- `static-world-mismatch`: zero observed.
- Later typed rejects included `steering-unreachable`,
  `progress-lift-rejected` and `velocity-unreachable`.
- Every record remained `authority=shadow,selected=0`; production Mission and
  normal command authority were unchanged.

A final post-cache run, `output/20260825-201010`, did not provide dynamic
evidence because AWSIM never supplied odometry and both controllers remained in
the startup missing-odometry failsafe. It is excluded from the causal result;
the final cache-only change is covered by the successful rebuild, source
contract and focused tests.

## Promotion conclusion

The current-world observation boundary is now structurally connected and the
static-world identity defect is closed. Gate A promotion is not yet accepted:
the selected async plan must explain or repair current actuation/progress
reachability, and Pass coverage has not been observed. The next Slice must
audit those producer-to-adoption time/coordinate contracts before deleting the
five-state Gate A.
