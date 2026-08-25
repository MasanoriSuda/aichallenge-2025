# Validation

## Evidence boundary

- Branch: `develop_july`
- Baseline: `cd69a23 refactor(mpcc): remove retired formulation switch state`
- Structural dynamic reference: `output/20260825-112734`
- User-owned `aichallenge/result-summary.json` is excluded.

## Pre-fix producer audit

- `pending_dynamic_escape_execution_` has no assignment anywhere in the
  production controller; it is only reset or read.
- `retained_dynamic_escape_execution_ = pending` exists only inside
  `accept_current_dynamic_escape_execution()`, so the retained store cannot be
  created without the absent pending producer.
- `dynamic_obstacle_lateral_escape_formulation_lease_until_sec_` is assigned
  only negative infinity during reset/retirement; it can never activate.
- The accepted structural run contains no `retained-stage`,
  `retained-execution` or retained DynamicEscape publication evidence.

## Failure-first proof

- Added `test_unproducible_retained_dynamic_escape_path_is_physically_deleted`.
- Before implementation it failed as intended while 36 existing source
  contracts passed: `1 failed, 36 passed`.
- After deletion: `37 passed in 0.54s`.

## Static validation

- Exact-symbol search finds no private DynamicEscape retained store, cursor,
  lease, restore/promote helper, retained-only exit reason or
  `published_source=retained-stage` in production or C++ tests.
- `git diff --check`: pass.
- `make autoware-build`: 25 packages passed.
- Rebuilt with `BUILD_TESTING=ON`: pass.
- Full `multi_purpose_mpc_ros` test suite: 1,861 tests, 0 errors, 0 failures,
  0 skipped.
- The seven removed tests exercised only the deleted cursor/lease contract;
  live DynamicEscape attempt, exit, replacement and wall-admission tests remain.

## Dynamic validation

The deletion is behavior-neutral for the reachable graph: current production
already takes the fresh-only branch, and `output/20260825-112734` contains no
retained publication evidence. Fresh candidate construction and its physical
wall admission were not altered. A new dynamic acceptance run is therefore not
required for this deletion Slice; it remains required for any later authority
promotion or fresh wall-admission change.
