# Validation

## Static validation

- `git diff --check`: passed.
- `python3 -m pytest test/test_single_authority_source_contract.py -q`:
  28 passed.
- `make autoware-build`: passed, 25 packages built.
- Container package tests: 48 test targets, 1874 tests, 0 failures,
  0 errors and 0 skipped tests.
- `colcon test-result --verbose` also printed an unrelated stale build-tree
  traceback for `joycon_contract_guard/package.xml`; the test summary itself
  remained 1874 tests with zero failures/errors.

## Dynamic validation

### Two-vehicle gate

Run: `make dev2`

Artifact: `output/20260825-061048`

- Both d1 and d2 emitted `Rate-resolved retained current-world shadow`.
- All observed retained attempts failed closed as
  `dynamic-observation-unavailable`; no retained command was selected.
- Typical evaluator runtime was about 0.01--0.02 ms.  The maximum observed
  telemetry-window sample was 1.071 ms.
- d2 recorded one 25 ms callback overrun.  That window reported
  `mpc_ms=5.996/20.552(avg/max)` while the retained evaluator's window was
  materially smaller.  Similar callback overruns exist in pre-Slice runs, so
  this is not evidence of a retained-proof regression.

### Single-vehicle diagnostic

Run: `make dev`

Artifact: `output/20260825-061340`

- The shadow path remained fail-closed with
  `dynamic-observation-unavailable`.
- The result establishes that the current V2X producer does not necessarily
  publish an explicit, current zero-vehicle observation even in a single-car
  run.  Absence of a message is deliberately not interpreted as an empty
  dynamic world.

## Conclusion

The Slice closes the missing static-source provenance and current-state join
boundary without changing authority.  Production promotion is still blocked:
dynamic-world evidence needs an explicit empty-observation contract or a
current obstacle-tube proof.  That work must be a separate root-cause Slice;
loosening this evaluator would turn retained admission into an age-only
fallback.
