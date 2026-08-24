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

### Run 1: `output/20260825-020710`

- D1: 399 consumed / 399 solved
- D2: 6,959 consumed / 6,959 solved
- Combined: 7,358 solved; build/assembly/solve/nonfinite/sample/exception
  rejects all zero
- 11 publication samples crossed into stage one; sampled maximum stage was one
- Mailbox invalid/rollback/unsubmitted: zero
- Every one of 94 aggregate records remained
  `authority=shadow, selected=0`
- Maximum rate-resolved compute / solve time: 11.645 / 11.576 ms

### Run 2: `output/20260825-021144`

- D1: 396 consumed / 396 solved
- D2: 6,328 consumed / 6,328 solved
- Combined: 6,724 solved; build/assembly/solve/nonfinite/sample/exception
  rejects all zero
- 11 publication samples crossed into stage one; sampled maximum stage was one
- Mailbox invalid/rollback/unsubmitted: zero
- Every one of 86 aggregate records remained
  `authority=shadow, selected=0`
- Maximum rate-resolved compute / solve time: 10.325 / 10.246 ms

### Result

Across both runs, all 14,082 consumed results solved and sampled successfully.
The historical rare `SolveRejected` did not recur, so the new failure trace was
`NOT EXERCISED` dynamically. This does not prove that the historical failure is
fixed; it proves that the observation path is installed without changing
authority and that no further behavioral repair is justified without evidence.

Each run had one independent 25 ms control-callback overrun (25.644 ms and
27.513 ms). The first coincided with initial cold production solve activity; the
second occurred during normal Track/Cruise with a nearby production certificate
window whose maximum certificate time was 19.225 ms. The rate-resolved worker
itself remained asynchronous and below 11.645 ms. Callback attribution is a
separate production-quality blocker and is not repaired in this diagnostic
Slice.
