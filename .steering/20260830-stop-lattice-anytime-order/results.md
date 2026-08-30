# Results

## Implemented search schedule

The Stop evaluator now consumes an anytime permutation of the existing
deterministic lattice.  It does not add, remove or modify a steering-rate
schedule.  The legacy sign-major population remains available as the audit
reference.

The anytime permutation:

- starts from the geometry nearest the existing `(0.15, 0.30)` switch
  fractions;
- traverses remaining geometries with deterministic normalized
  farthest-point coverage;
- places positive and negative initial steering-rate signs next to each other;
- puts the sign continuous with the selected normal command first, or unwinds
  current steering when the preceding rate is zero;
- records the one-based legacy rank for every permuted member.

The first certified candidate still must pass the unchanged seven-state solve,
exact nonlinear trajectory, rest, exact wall, all-peer current-world and
certified-plan chain.

## Static evidence

- anytime and legacy populations have exact one-to-one set equality;
- no legacy rank is duplicated or omitted;
- both signs are adjacent for each accepted geometry;
- the first geometry is nominal and the next geometry maximizes coverage
  distance from it;
- repeated construction is deterministic;
- negative publisher-boundary steering-rate continuity selects the negative
  sign first;
- live result telemetry now reports anytime rank, legacy rank, population size
  and preferred sign;
- observation authority remains `shadow, selected=0` and no Store/publisher
  edge was added.

## Verification

- `make autoware-build`: passed (25 packages);
- full `multi_purpose_mpc_ros` CTest: 59 / 59 passed;
- aggregate tests: 2274, 0 errors, 0 failures;
- single-authority source contract: 92 / 92 passed in the Docker suite;
- `git diff --check`: passed;
- production parameters, certificate policy and authority: unchanged.

The host-only pytest invocation was not used as evidence because the ROS launch
plugin collected the package directory and hit an unrelated missing
`localization_scope` import.  The same test ran successfully in the supported
Docker/colcon environment.

## Remaining dynamic gate

Run the same `make dev2` scenario and compare `rank=anytime:legacy:population`
and result age against `output/20260830-222744`.  Acceptance requires lower
rank/age for opposite-sign and broad-shape successes without increased
callback tail or a production behavior change.  A difficult epoch that still
requires the final schedule remains evidence for a later cancellation or
candidate-generation Slice, not permission to add a timeout here.

## Live comparison

The statically accepted Slice was exercised with `make dev2`.  The immutable
run evidence is under:

`output/20260830-224528/d1/autoware.log`

Production authority remained unchanged.  The run reached five ShiftOut
entries and all five entered Pass.  Two reached Return and completed the
Return-to-Idle handoff.  One Pass entered Recovery after longitudinal progress
stalled and two entered the existing Dynamic Mission wait.  No `actual
footprint wall margin violated` message was observed.  These production
transitions are context only: the Stop lattice remained
`authority=shadow, selected=0` throughout the run.

The shadow comparison produced:

| Evidence | Baseline `222744` | Anytime `224528` |
|---|---:|---:|
| submitted / replaced | 64 / 35 | 49 / 14 |
| started / completed | 29 / 29 | 35 / 35 |
| mailbox published / invalid / rollback | 29 / 0 / 0 | 35 / 0 / 0 |
| interval-consumed / accepted | 35 / 24 | 35 / 29 |
| solver rejections | 5 | 6 |
| maximum complete evaluation | 4158.885 ms | 3655.939 ms |
| maximum consumed result age | 5.3300 s | 5.4950 s |
| maximum candidate attempts | 68 | 68 |
| worker exceptions | 0 | 0 |

The common opposite-sign Stop schedule is now evaluated first instead of
forty-second.  For example, schedule `-1:3:6` was accepted at anytime rank 1,
legacy rank 42 with complete evaluation times of 45.806--80.748 ms in several
live ShiftOut states.  The adjacent sign also appeared at rank 2 instead of
requiring a second sign-major sweep.  This confirms the intended common-case
freshness improvement without changing the physical or certificate policy.

The hard tail remains.  Six epochs exhausted all 68 candidates and reported
the unchanged seven-state maximum-iteration rejection.  Because the worker
finishes the obsolete running epoch before starting the newest pending epoch,
one later result was already 5.4950 seconds old when consumed.  The control
callback also recorded a 211.990 ms maximum and 160 total overrun cycles in
this longer run.  The Stop evaluator is on its own worker, so the run does not
prove callback causation, but it fails the required no-tail-regression gate.

## Dynamic conclusion

The Slice is accepted as a complete-set scheduling improvement and rejected
for production promotion.  It proves that ordering, not a different solver or
clearance, removes the common legacy-rank-42 latency.  It also proves that
ordering alone cannot bound freshness: an obsolete infeasible epoch may still
consume the complete lattice while a newer source waits.

The next root-cause Slice must audit cooperative supersession between
candidate solves.  It may abandon observation work whose source sequence is
already obsolete, but it may not add a Mission timeout, lease, grace period,
fallback, solver tolerance change, clearance change or production authority
edge.  Candidate generation remains unchanged until that scheduling
hypothesis is tested.
