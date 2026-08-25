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

A committed-source `make dev2` trial was run as
`output/20260825-112734` on `62d0316`.

Structural acceptance passed in both domains:

- zero `legacy-mpc-solved`, `extended-mpcc-solved`,
  `legacy-spatial-mpc-3state` or `progress-contouring-3state` normal owner;
- zero `canonical normal intent has no production owner`;
- zero Track/Cruise or Rejoin admission-fallthrough diagnostic;
- zero `canonical normal command mutated before publication`;
- Track/Cruise traces retained `velocity-steering-progress-6state`; Follow
  traces retained `velocity-progress-5state`;
- callback telemetry reported zero 25 ms overruns in the observed windows.

The run did not exercise ShiftOut, Pass or Return, so it does not establish
stopped/slow-vehicle acceptance. It also exposed a pre-existing six-state
Track/Cruise quality failure which is outside this deletion Slice. The first
d1 abnormal cycle was at `1787624899.601839468`: the preceding six-state solve,
physical wall certificate and exact publication had all been accepted, but the
current vehicle state reached `e_y=-2.072 m` and the fail-operational crawl was
rejected as path-unsafe. The controller then emitted the typed
`canonical-cruise-emergency/rate-resolved authority unavailable/
retained-proof-unavailable` result and Recovery later became active.

This is not evidence that removing the lexical legacy fallback caused a solve
or formulation regression. The accepted promotion baseline
`output/20260825-100454` already contains the same crawl-block diagnostic three
times in each domain, and `62d0316` does not change the live six-state solver,
certificate or production adapter. It is evidence that six-state
Track/Cruise execution quality is not yet dynamically accepted for sustained
racing. That issue must be audited at the six-state producer/current-world
proof boundary and must not be masked by restoring the deleted normal
formulation.

## Residual debt found, not patched

The old normal solver deletion makes several migration-only objects inert:

- extended circuit/reentry/handoff telemetry used by the old live solve;
- retained legacy dynamic-escape execution and wall-handoff lease graph;
- historical `shadow` names on promoted canonical owners.

These are explicit subsequent physical-deletion Slices. They were not removed
opportunistically here because final publisher wall/recovery arbitration must
be separated from normal solver ownership before deleting shared state.
