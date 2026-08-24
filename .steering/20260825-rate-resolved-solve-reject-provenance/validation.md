# Validation

## Static checks

- `git diff --check`: PASS
- `make autoware-build`: PASS
  - 25 packages completed
  - only the pre-existing setuptools deprecation warning was emitted
- Full `multi_purpose_mpc_ros` package test: PASS
  - 44 test targets
  - 1,832 tests
  - 0 errors, 0 failures, 0 skipped
- `test_single_authority_source_contract.py`: 24 passed as part of the full
  package run

## Host-only pytest attempt

Direct host collection was not a valid package test environment because the
package-local `localization_scope` module was not installed. The authoritative
Docker/colcon run loaded the package overlay and passed all tests.

## Dynamic validation

Pending. Run `make dev2`, then confirm one of:

- a non-solved result emits `Rate-resolved Track/Cruise shadow failure` with
  exact solver detail; or
- no non-solved result occurs, recorded explicitly as `NOT EXERCISED`.
