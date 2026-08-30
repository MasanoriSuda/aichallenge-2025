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
