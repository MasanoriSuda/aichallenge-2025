# Results

## Static verification

- `make autoware-build`: 25 packages passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: all 52 tests passed;
  2072 test cases reported with zero errors and zero failures.
- The source contract proves that the audit arm cannot store, mark executed or
  publish its result.

## Dynamic A/B

Run: `output/20260829-003210/d1/autoware.log`

At decision 615 the normal asynchronous path had no plan and no authority:

```text
async_reason=missing-plan
async_candidate=missing-plan/0
async_executed=missing-plan/0
```

The same observation, exact last serialized predecessor, seven-state SQP,
exact physical proof and current-world proof succeeded synchronously:

```text
sync_solver=solved
sync_physical=accepted
sync_world=accepted
sync_authority=1
sync_solver_ms=78.129
sync_total_ms=81.461
```

The direct result remained observation-only.  Production still emitted the
existing emergency command for that cycle.

## Classification

The frozen failure is not physical infeasibility and not an absence of a
seven-state solution.  Normal-control authority is lost because the main
solve is produced asynchronously and must later join a different serialized
predecessor and world state.  This agrees with the upper-rank log: its main
GMPCC is solved directly, while only tactical alternatives are moved to a
child process.

The structural correction is to make the direct seven-state solve the sole
normal producer, retain only the last actually published certified artifact,
and remove asynchronous normal candidate adoption.  Asynchronous tactical
left/right work remains separate.
