# Validation

## Static checks

- `git diff --check`: PASS
- changed Python source-contract module compiles: PASS
- `make autoware-build`: PASS
  - 25 packages completed
  - only the pre-existing setuptools deprecation warning was emitted
- full `multi_purpose_mpc_ros` package test: PASS
  - 1,837 tests
  - 0 errors, 0 failures, 0 skipped
  - the stale `joycon_contract_guard/package.xml` parser warning remains
    unrelated to the selected package

## Contract coverage

- Normal and clearly moving: safety map work is not required.
- Normal, low speed and forward intent: full safety remains required.
- Suspect/active Recovery: full safety remains required.
- solver fallback, rearm guard and dynamic lateral execution: full safety
  remains required.
- invalid speed/config values: fail closed into full safety.
- source contract proves the same eligibility result gates both preliminary
  wall classification and full safety evaluation, while
  `StuckRecoveryCore::update()` remains unconditional.

## Dynamic validation

Pending committed-source dev2 execution.
