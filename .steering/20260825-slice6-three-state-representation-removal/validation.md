# Validation

## Failure-first contract

With the deletion contract added before implementation:

```text
1 failed, 37 passed
```

The failure listed both retired formulation values, their string/schema identities and the lossy
conversion API. After implementation:

```text
38 passed
```

## Reachability proof

- `LegacySpatialMpc3State`: zero production producers; enum, string and schema switch only.
- `ProgressContouring3State`: zero production producers; enum, string, schema switch and two
  manufactured rejection-test inputs only.
- `convert_extended_solution_to_legacy()`: one declaration, one definition, zero production call
  sites and two dedicated tests.

Static search after deletion found none of the retired symbols or schema strings in production
headers or sources.

## Build and tests

- `make autoware-build`: 25 packages completed successfully.
- Test-enabled clean package build: completed successfully.
- `colcon test --packages-select multi_purpose_mpc_ros`: 49/49 targets passed.
- `colcon test-result --verbose`: 1,821 tests, zero errors, failures or skips.
- `git diff --check`: passed.

The build emitted only the existing ROS 2 scoped-header-install recommendation and setuptools
deprecation warnings.

The two conversion-only tests were deleted and one physical-deletion source contract was added, so
the package total decreased by one test from the preceding Slice. Noncanonical formulation
rejection remains covered using the live `SolverDerivedBypass` exceptional formulation.

## Dynamic replay decision

No dynamic replay was required. The removed representations had no runtime producer and the
converter had no production call site. The five-state and six-state solvers, Rejoin, publication,
Emergency and Recovery paths are unchanged.

## Acceptance

Accepted as a behavior-neutral Slice 6 deletion. A retired normal solver can no longer be
reconnected through execution-contract identity or a lossy solution-layout adapter.
