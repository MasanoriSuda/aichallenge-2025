# Validation

## Evidence boundary

- Branch: `develop_july`
- Baseline: `4b89fcd refactor(mpcc): remove five-state track cruise owner`
- This Slice changes the final normal dispatch boundary, but does not change
  parameters, ROS interfaces, Recovery or canonical solver formulations.
- User-owned `aichallenge/result-summary.json` was neither edited nor staged.

## Observed architecture defect

All accepted normal intents already returned through canonical Track/Cruise,
Follow, Overtake, Rejoin or Stop branches. After those branches,
`MPC::get_control()` still contained a synchronous execution chain:

```text
extended five-state solve
  -> circuit / reentry / handoff
  -> three-state progress fallback
  -> legacy spatial MPC fallback
  -> postprocessed normal command
```

This was reachable whenever an intent was unresolved or a migration
eligibility prerequisite was false. A failed admission could therefore change
controller formulation instead of failing within the same authority contract.

## Failure-first proof

The new source-contract assertion failed with one failure and 34 passes before
the implementation because `Eigen::VectorXd dec`, `solve_problem()` and the
legacy/extended resolution strings remained in `get_control()`.

After the structural deletion, the source-contract suite reports 35 passes and
rejects restoration of:

- legacy or progress-three-state formulation inside normal dispatch;
- synchronous extended solve and five-to-three-state command conversion;
- `solve_problem()` or its private persistent solver history;
- a normal intent without either canonical owner or explicit Emergency.

## Structural change

- Dispatch Track/Cruise and Rejoin by resolved intent. Their eligibility flags
  now prove admission inside that owner; false eligibility cannot select an
  older formulation.
- Unsupported/unknown normal intent returns explicit canonical Emergency.
- Removed the complete post-Rejoin synchronous normal solve block.
- Removed `solve_problem()`, its persistent solver, warm plan, age and
  formulation/progress history.
- Removed stale dynamic-escape trace state that could report `progress-3state`
  or `legacy-mpc`; traces now use the actual canonical problem context.
- Added no fallback, feature flag, lease, timeout, clamp or parameter.

## Static validation

- `test_single_authority_source_contract.py`: 35 passed.
- `make autoware-build`: 25 packages passed.
- Package rebuilt with `BUILD_TESTING=ON`.
- Package CTest: 49 targets passed.
- `colcon test-result --verbose`: 1,870 tests, zero errors, failures or skips.
- `git diff --check`: passed.

## Dynamic acceptance

Pending a short committed-source `make dev2` trial. Required observations:

- no `legacy-mpc-solved`, `extended-mpcc-solved` or three-state normal owner;
- Track/Cruise, Follow and Overtake publish canonical identity or a typed
  canonical Emergency reason;
- no unexpected `canonical normal intent has no production owner`;
- no Emergency burst at DynamicEscape-to-Track/Cruise transition;
- no regression in stopped/slow vehicle avoidance.

## Residual debt found, not patched

The old normal solver deletion makes several migration-only objects inert:

- extended circuit/reentry/handoff telemetry used by the old live solve;
- retained legacy dynamic-escape execution and wall-handoff lease graph;
- historical `shadow` names on promoted canonical owners.

These are explicit subsequent physical-deletion Slices. They were not removed
opportunistically here because final publisher wall/recovery arbitration must
be separated from normal solver ownership before deleting shared state.
