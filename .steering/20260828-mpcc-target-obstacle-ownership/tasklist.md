# Task list

- [x] Separate production and offline-generated architecture snapshots.
- [x] Re-run A/B/C/D/F comparison on the production snapshot.
- [x] Falsify the standalone automatic-diagonal timing hypothesis.
- [x] Trace the missing target constraint to the exclusion-release contract.
- [x] Add failing contract/source tests.
- [x] Remove exclusion-based target release.
- [x] Run focused tests and full build/tests.
- [x] Update migration evidence and commit.
- [ ] Run dynamic acceptance and classify any remaining wall-first failure.

## Validation

- `make autoware-build`: passed (25 packages).
- `test_overtake_execution_orchestrator --gtest_filter=*DynamicObstacleContract*`:
  3/3 passed.
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest
  test/test_single_authority_source_contract.py -q`: 66/66 passed.
- `colcon test-result --verbose`: 2045 tests, 0 errors, 0 failures.
  The result reader emitted an unrelated stale
  `build/joycon_contract_guard/package.xml` warning.

The remaining dynamic-acceptance item is deliberately left open: static and
replay tests prove the contract ownership repair, but a fresh production run
must show a non-empty target tube on an equivalent `ShiftOut`/`Pass` decision.
