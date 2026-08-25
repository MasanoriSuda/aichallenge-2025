# Validation

## Static

- `git diff --check`: pass.
- failure-first source contract: pass.
- all single-authority source contracts: 50 passed.
- `make autoware-build`: 25 packages passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 49/49 targets,
  1,865 tests, 0 failures. `colcon test-result` still reports the pre-existing
  stale `joycon_contract_guard/package.xml` result path, but returns a zero
  test-failure summary.

## Dynamic

Bounded two-vehicle run: `output/20260825-184710`.

Domain 1 recorded:

- 60 throttled telemetry windows;
- 13 windows with a prospective six-state side attempted;
- 8 windows with a solved six-state artifact;
- 4 windows with solver, swept static-wall and target-tube proof all accepted;
- zero six-state shadow selections (`authority=shadow,selected=0`).

Examples:

- line 453: left ShiftOut solved in 1.31 ms, physical proof in 2.90 ms,
  terminal progress 18.74 m and all three proof stages accepted;
- line 474: solver and wall accepted, but target proof rejected at sample 11.
  The shadow therefore distinguishes target infeasibility from solver/wall
  failure instead of collapsing them into one Gate-A result;
- early samples at lines 314/323/337 failed at the six-state solver with
  maximum iterations and remained observation-only.

Domain 2 had no relevant Overtake candidate and therefore no six-state solve.
After suppressing not-attempted-only telemetry, this domain will not emit a
second empty log line.

The existing production path later entered ShiftOut using the five-state Gate
A. Final normal execution remained the established six-state authority;
explicit Emergency remained `formulation=unresolved`. The shadow result never
changed branch selection, Mission state or final publication.

## Exit decision

The shadow Slice passes. Promotion is deliberately blocked: only ShiftOut was
observed, four complete samples are insufficient acceptance coverage, and
Pass/Return prospective intent was not exercised. The next Slice must compare
five-state and six-state Gate-A acceptance by immutable observation identity,
then promote six-state and delete five-state only when the dynamic gate is
satisfied.
