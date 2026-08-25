# Requirements

## Goal

Make a canonical intent transition one traceable admission transaction.  A
six-state solve and immutable physical proof are necessary, but they are not
evidence that the exact plan is joinable to the current command and dynamic
world.

## Evidence boundary

- baseline HEAD: `83edbba`
- moving evidence: `output/20260826-055949/d2/autoware.log`
- incident: decision 2140 changes `Pass -> Return`, synchronously certifies
  sequence 1551, then publishes `retained-proof-unavailable` emergency
- subsequent cycles repeat synchronous Return solves and lose normal authority

## Constraints

- Do not tune solver, vehicle, wall, speed or clearance parameters.
- Do not add a fallback, lease, timeout or alternate normal authority.
- The exact physically certified plan must be the plan evaluated against the
  current world; do not rediscover it from a mutable newest-candidate slot.
- A physically certified but current-world-rejected plan must remain fail
  closed and expose the typed rejection reason.
- Preserve ROS and evaluation interfaces and the user-owned
  `aichallenge/result-summary.json` modification.

## Definition of done

- Transition admission reports physical certification and production join as
  separate facts.
- The exact admitted sequence is used for current-world evaluation.
- Production consumes that exact evaluation without a second mutable-store
  lookup.
- Failure-first source/contract tests prevent a return to
  `certified -> rescan store -> unexplained emergency`.
- Package build and complete tests pass.
- A moving run records the exact Return join rejection or successful Return
  publication before any tactical-state promotion work continues.
