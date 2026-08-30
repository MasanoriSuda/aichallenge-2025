# Results

## Implemented observation boundary

The exact ShiftOut/Pass snapshot selected by the normal asynchronous worker is
carried with its certified normal artifact.  A Stop comparison is submitted
only after the existing Store contains that same immutable artifact identity.
It runs on a distinct latest-only worker and private solver context.

The comparison performs:

1. publisher-boundary rebasing;
2. deterministic bounded steering-rate lattice enumeration;
3. the same seven-state solve;
4. exact nonlinear trajectory adaptation;
5. exact wall and all-peer current-world proof;
6. certified-plan validation.

The resulting plan enters only a monotonic observation mailbox.  It has no
Store replacement, production-adapter or publisher callsite.  Bounded debug
telemetry reports worker replacement, age, candidate attempts, solver time,
exact reserves and rejection reason with `authority=shadow, selected=0`.

## Candidate-source defects exposed by the positive test

The first complete live-evaluator fixture rejected before the solver because
publisher-boundary rebasing updated `request.initial_state` without updating
the stage-zero fixed state box.  The semantic adapter correctly reported
`initial-state-outside-bounds`.  The shared candidate source now rebases the
state-zero reference/lower/upper equality atomically.

The second fixture solved the full Stop horizon but exposed only the normal
short execution prefix, so exact proof observed a still-moving artifact.  A
Stop successor now explicitly exposes its complete horizon through the
execution artifact.  This is necessary to prove rest and is not a solver or
clearance adjustment.

## Frozen replay after the invariant fixes

Both frozen failures remain feasible with unchanged solver/clearance settings:

| Snapshot | Result | Candidate | Exact lateral reserve |
|---|---|---|---:|
| decision 4017, ShiftOut positive | accepted | positive 3 / negative 3 / hold, #8 | 0.0342699 m |
| decision 4489, Pass negative | accepted | positive 3 / negative 3 / hold, #8 | 0.0429072 m |

The reserves differ from the earlier short-prefix audit because exact proof
now covers the complete Stop trajectory rather than only the normal publisher
prefix.  Both terminal velocities are approximately zero.

## Static verification

- positive evaluator fixture: solve, exact trajectory, wall, all-peer and
  certified-plan chain accepted;
- identity mismatch: rejected before enumeration;
- mailbox sequence rollback: rejected;
- source-level authority test: no Store/publisher/production edge;
- frozen decision 4017 and 4489 terminal audits: accepted;
- `make autoware-build`: passed (25 packages);
- full `multi_purpose_mpc_ros` CTest: 59 / 59 passed;
- `git diff --check`: passed;
- production parameters and authority: unchanged.

## Remaining dynamic gate

Live Overtake evidence is still required.  The next run must show whether the
separate worker keeps up with ShiftOut/Pass epochs, how often pending work is
replaced, and whether accepted Stop observations exist before production
authority loss.  No production promotion is authorized by this Slice.
