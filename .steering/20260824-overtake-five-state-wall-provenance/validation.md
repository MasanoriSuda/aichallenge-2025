# Validation

## Static and unit validation

- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q .../test_single_authority_source_contract.py`
  - Result: `5 passed`.
  - Proves branch and live wall proof occur before legacy conversion and entry
    keeps exact certificate identity.
- `make autoware-build`
  - Result: 25 packages built successfully.
- Container package test:
  - `colcon test --packages-select multi_purpose_mpc_ros`
  - Result: 40/40 CTest targets passed; 1,731 tests, zero failures/errors.
  - `colcon test-result --verbose` also printed an unrelated missing generated
    `joycon_contract_guard/package.xml` traceback after the clean test summary;
    it did not fail this package run.
- `git diff --check`
  - Result: clean.

## Dynamic validation

- Command: `make dev2`
- Output: `output/20260824-063046`
- Domain 1 entered Overtake once with a complete entry certificate.
- First live ShiftOut solution:
  - exact five-state swept proof accepted;
  - normal command published;
  - solver status `extended-mpcc-solved`.
- Later live ShiftOut solution:
  - exact proof rejected before command adaptation;
  - exact diagnostic identified stage 15, waypoint 201, lateral, lag, heading
    and solved/reference progress delta;
  - existing fail-closed wall action was used.
- No Overtake entry was admitted from a missing/lateral-only certificate.

## Interpretation

The Slice meets its acceptance criteria: one exact five-state artifact owns
physical proof through branch, admission and live solve. The run also exposed a
separate continuous swept-wall feasibility gap. It is recorded as the next
root-cause Slice and is intentionally not patched here.

## Unrelated workspace state

`aichallenge/result-summary.json` was modified by the user's run and is not part
of this Slice or its commit.
