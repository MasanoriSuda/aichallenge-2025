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

## Live Overtake observation

The accepted Slice was exercised with `make dev2` and then stopped cleanly.
The immutable run evidence is under:

`output/20260830-222744/d1/autoware.log`

The run reached five ShiftOut entries, four Pass entries, three Return entries
and three completed Return-to-Idle handoffs.  One ShiftOut and one Pass moved
to `FollowPrepare` through the existing Dynamic Mission wait.  There were no
`actual footprint wall margin violated` messages.  Production authority and
all normal transition behavior remained unchanged by the shadow.

The Stop shadow exercised the intended live boundary:

| Evidence | Observed value |
|---|---:|
| submitted / replaced | 64 / 35 |
| started / completed | 29 / 29 |
| mailbox published / invalid / rollback | 29 / 0 / 0 |
| interval-consumed / accepted observations | 35 / 24 |
| solver rejections reported by interval telemetry | 5 |
| maximum complete lattice evaluation | 4158.885 ms |
| maximum consumed result age | 5.3300 s |
| worker exceptions | 0 |

Accepted observations covered both ShiftOut and Pass.  They passed exact
nonlinear trajectory, rest, wall, all-peer dynamic and certified-plan checks.
Examples include:

- ShiftOut: schedule `1:3:6`, exact lateral reserve `0.0639 m`, dynamic
  reserve `2.0448 m`;
- Pass: schedule `1:6:7`, exact lateral reserve `0.0011 m`, dynamic reserve
  `3.0646 m`;
- opposite-side ShiftOut: schedule `-1:3:6`, exact lateral reserve
  `0.0474 m`, dynamic reserve `4.5091 m`.

The only classified evaluation rejection in the live telemetry was the
seven-state solver reaching its iteration limit.  The later exact-wall,
dynamic and certified-plan stages did not reject a solved candidate.

## Dynamic conclusion

The live gate is accepted for observation, not for production promotion.
The result disproves the hypothesis that a physically certified Stop suffix
cannot be produced from live ShiftOut/Pass states.  It also isolates the next
problem: exhaustive deterministic enumeration is not fresh enough for a
production successor.  Although individual selected solves were generally
tens of milliseconds, evaluating up to 68 candidates made complete results
2.6--5.3 seconds old in difficult epochs.

The next Slice must therefore preserve the same certificate chain while
changing scheduling/search semantics: return the first sufficiently robust
certified Stop candidate under an explicit live deadline, cancel or abandon
the remaining obsolete search, and keep a later result observationally
distinct from current authority.  Solver tolerances, clearance margins and
production authority remain frozen until that freshness question is answered.

Control callback overruns were also observed (a worst logged callback of
`58.191 ms` against the `25 ms` period, and a one-second window with ten
overruns).  The Stop computation ran on its separate worker, so this run does
not by itself prove causation.  Any production proposal must demonstrate that
its deadline/cancellation policy does not increase callback tail latency.
