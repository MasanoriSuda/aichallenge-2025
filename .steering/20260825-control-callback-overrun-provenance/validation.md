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

Pending dev2 execution.
