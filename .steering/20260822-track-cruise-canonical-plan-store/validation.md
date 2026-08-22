# Validation

## Failure-first expectation

The baseline has no type that can distinguish a complete executable five-state plan from solver
metadata plus a stage count. Compiling deterministic lifecycle tests must fail before the contract
is implemented.

Observed failure after CMake regeneration:

```text
fatal error: multi_purpose_mpc_ros/canonical_execution_plan.hpp:
No such file or directory
```

The failure occurs before runtime wiring and demonstrates that the baseline had no complete-plan
lifecycle contract.

## Implemented invariants

- `CanonicalExecutionPlan` stores exactly `N + 1` five-state predictions and `N` three-input stages.
- Zero-length, partial, non-finite, noncanonical and uncertified plans fail explicitly.
- The thread-safe store replaces the immutable snapshot only after complete validation and only for
  a newer nonzero plan ID.
- A delayed callback cannot clear a newer plan because clearing is compare-by-plan-ID.
- The accepted plan-ID high-water mark survives clearing, so a delayed older solve cannot repopulate
  an intentionally empty store.
- The time cursor advances by exact stage durations and returns `Exhausted` instead of repeating the
  final input.
- Candidate construction requires current decision, plan ID, first stage, remaining count and
  physical wall/obstacle proof to match.
- The final canonical authority selector independently rejects a physically uncertified execution
  proof.

## Focused verification

- `test_canonical_execution_plan`: 8/8 passed.
- `test_mpcc_execution_contract`: 37/37 passed.
- An initial empty-horizon test exposed an ambiguous `IncompleteProblem` reason. Validation order was
  corrected so the trace now reports `empty-horizon` explicitly.

## Build and regression

- `make autoware-build`: 25 packages built successfully.
- `colcon test --packages-select multi_purpose_mpc_ros`: 34/34 CTest targets passed.
- `colcon test-result --verbose`: 1,589 tests, zero errors, failures or skips.
- The existing stale `build/joycon_contract_guard/package.xml` result-parser warning remains; the
  command exited successfully and no package test failed.
- `git diff --check`: passed.

## Runtime status

This Slice is deliberately runtime-disconnected. Dynamic simulation cannot show a command change;
its purpose is to make the later authority connection structurally safe and auditable.
