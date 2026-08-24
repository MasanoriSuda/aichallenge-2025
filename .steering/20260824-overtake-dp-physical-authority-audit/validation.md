# Validation

## Static checks

- `git diff --check`: passed.
- focused source-contract tests: 14 passed.
- `make autoware-build`: passed, 25 packages completed.
- package CTest in the development container: 40/40 passed.
- `colcon test-result --verbose`: 1760 tests, 0 errors, 0 failures,
  0 skipped.

The test-result scan also emitted the pre-existing unrelated warning that
`joycon_contract_guard/package.xml` is absent.

Host `clang-format` was not available. Formatting was nevertheless exercised
by the package build/test checks that are present in the repository.

## Dynamic check

Command: bounded `make dev2` run.

Artifact: `output/20260824-130017`

The run produced the intended hard-failure trace and separated the events
which were previously aggregated:

- canonical current-world initial corridor rejection;
- explicit emergency override;
- fresh generation-1 DP authority;
- independent receding physical failure;
- legacy Mission invalidation and Recovery.

No production threshold, state transition, solver input, or command owner was
changed by this Slice.
