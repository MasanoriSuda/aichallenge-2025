# Requirements: Cruise current-world avoidance population

## Repaired invariant

A normal dynamic-obstacle candidate may execute only after its complete
homotopy, dynamics, progress-dependent wall envelope and obstacle disjunction
have been solved and certified together.  Neutral Cruise/Follow must not turn
one obstacle-free witness into the only executable obstacle branch.

## Failure-first evidence

Frozen Cruise sequence 601 at commit `6884807f`:

- captured neutral automatic branch: solver rejected;
- independent nonlinear minimum slack: about `0.506597 m`;
- rebuilt positive side: all exact proofs accepted;
- rebuilt negative side: all exact proofs accepted.

The pre-fix production path calls the current-world side population for Follow
only.  Cruise with a dynamic obstacle falls through to the captured neutral
single-branch solve.

## Scope

- Share one current-world normal-avoidance population between Cruise and
  Follow when a current dynamic-obstacle contract is active.
- Keep intent and execution side neutral; this does not create Overtake
  authority or a Mission.
- Keep one persistent numerical owner per physical side and one bounded
  homotopy owner.
- Use the existing seven-state solve, exact wall/dynamic proofs, certified
  Store and normal publisher.
- Delete Follow-only producer/API naming and the production fall-through to
  neutral automatic obstacle refinement.

## Non-scope

- No solver, weight, clearance, margin, timeout, lease, retry or fallback
  change.
- No ShiftOut successive-convexification repair.
- No opposite-side switch after no-return Overtake commitment.
- No Emergency or reverse-Recovery change.

## Implementation gate

- rollback commit: `00f5f97b`;
- new production authority count: zero;
- removed production path: dynamic-obstacle Cruise direct neutral solve;
- failing replay: Cruise sequence 601 captured arm fails while both rebuilt
  sides certify;
- acceptance: focused tests, source contract, full package tests/build and the
  unchanged frozen comparison pass.
