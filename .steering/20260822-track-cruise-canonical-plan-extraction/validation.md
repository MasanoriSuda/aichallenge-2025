# Validation

## Failure-first expectation

The baseline has no direct adapter from the certified extended primal to
`CanonicalExecutionPlan`. A deterministic adapter test must fail to compile before implementation.

Observed after adding the test target and regenerating CMake against the baseline implementation:

```text
fatal error: multi_purpose_mpc_ros/canonical_execution_plan_adapter.hpp:
No such file or directory
```

This proves that the test initially depended on the missing production boundary rather than an
existing helper.

## Implemented contract

- Extracts all `N + 1` five-state predictions directly from the extended primal.
- Restores absolute progress exactly once from the solve-local origin.
- Extracts all `N` acceleration, curvature and virtual-progress inputs with exact stage duration.
- Rejects malformed dimensions, non-finite values, incomplete timing and mismatched provenance.
- Calls the canonical complete-plan validator before returning an accepted plan.
- Contains no call to `convert_extended_solution_to_legacy()`.

## Runtime status

The adapter remains runtime-disconnected in this Slice. It prepares a lossless input to the plan
store; it does not select or publish control.

## Verification

- Focused adapter test: 4/4 passed.
- `make autoware-build`: 25 packages passed.
- Complete package CTest: 35/35 passed.
- `colcon test-result --verbose`: 1594 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.

`colcon test-result` still prints the pre-existing stale
`build/joycon_contract_guard/package.xml` discovery warning. It does not create a test failure.
