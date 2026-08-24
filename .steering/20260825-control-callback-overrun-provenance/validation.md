# Validation

## Static checks

- `git diff --check`: PASS
- `python3 -m py_compile` for the changed source-contract test: PASS
- `make autoware-build`: PASS
  - 25 packages completed
  - only the pre-existing setuptools deprecation warning was emitted
- Full `multi_purpose_mpc_ros` package test: PASS
  - 44 test targets
  - 1,833 tests
  - 0 errors, 0 failures, 0 skipped
- `test_single_authority_source_contract.py`: 25 passed as part of the full
  package run

## Dynamic validation

### Diagnostic run 1

- Artifact: `output/20260825-022540`
- Control cycles: 19,337
- Maximum callback: 22.301 ms
- Callback overruns: 0
- Rate-resolved shadow: 9,370/9,370 solved
- Result: `NOT EXERCISED`; the instrumentation remained observation-only.

### Diagnostic run 2

- Artifact: `output/20260825-023027`
- Exact overrun: `d2/autoware.log:451`, decision 2096
- Total/budget: 26.098/25.000 ms
- Region ownership:
  - pre-MPCC: 0.055 ms
  - `MPC::get_control()`: 20.786 ms
  - post-MPCC: 0.020 ms
  - Recovery: 5.046 ms
  - publish/trace: 0.189 ms
  - unattributed: 0.002 ms
- Checkpoint: `complete`
- Behavior context: ordinary Cruise, no target, no Overtake authority and no
  Recovery command.
- Nearby production aggregate: 40/40 solved and certified, maximum solve
  13.880 ms, certificate 14.991 ms, canonical total 20.362 ms.

### Verdict

The dominant exact region is production `MPC::get_control()`, with synchronous
Recovery evaluation supplying the remaining period overrun. No solver,
authority, horizon, cadence or command setting was changed in this Slice.
