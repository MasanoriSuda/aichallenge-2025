# Results: bounded normal-avoidance population

## Root-cause result

Frozen decision `2451`, world fingerprint `883737710184574622`, classified the
failure as a candidate-generation defect:

- persistent Mission pipeline plus the seven-state SQP rejected;
- direct stateless positive and negative candidates rejected;
- a smooth negative schedule with the same world, model and certificates
  accepted;
- fixed-depth refinement did not improve the accepted depth-zero result.

The accepted schedule delayed full lateral commitment while decelerating.  No
hard constraint, wall clearance, solver tolerance, lease, timeout or Stop rule
had to change.

## Production replay

The production normal population was replayed against the same frozen
snapshot.  It evaluated at most four fresh candidates per side in anytime
order:

1. direct;
2. measured-steering-reachable schedule;
3. midpoint schedule when distinct;
4. boundary schedule when distinct.

The positive side rejected after four candidates.  The negative side accepted
`normal-boundary-lattice` at transition stage 18 and ahead stage 20.  Its
candidate fingerprint, `17513260188351363027`, exactly matched the accepted C
audit arm.  SQP, exact wall proof, dynamic-obstacle proof and terminal Stop
successor proof all accepted.

This closes the audit gap without adding another publisher, authority owner or
persistent path.  The population is rebuilt from each immutable world epoch;
normal intent remains side-neutral.

## Verification

- `make autoware-build`: passed, 25 packages built.
- focused stateless maneuver tests: 29/29 passed.
- focused architecture comparison tests: 34/34 passed.
- complete `multi_purpose_mpc_ros` CTest suite: 59/59 passed.
- `git diff --check`: passed.

## Dynamic acceptance still required

The frozen evidence proves the structural repair but is not a race acceptance
test.  The next dynamic trial must verify:

- `terminal-contingency-unavailable` falls only when every bounded member is
  physically uncertifiable;
- accepted normal candidates report their exact kind, count and primary or
  sibling origin;
- direct candidates remain the common fast path;
- all-member failure still delegates to the existing certified Stop path;
- callback p95/p99 and background tails do not regress materially.
