# Design: Rough disjunction lattice candidate C

## Candidate representation

For horizon `N`, one lattice member selects:

- pass side `{-1,+1}`;
- first full-side stage `k in [0,N-1]`;
- first rear-clear/ahead stage `j in [k+1,N]`;
- stages `< k`: exact longitudinal stay-behind disjunct;
- stages `k <= stage < j`: exact full lateral-separation disjunct;
- stages `>= j`: exact longitudinal-ahead disjunct.

The lateral reference is a smoothstep from current lateral position to the
stage-wise full-side boundary by state `k+1`, follows it through rear-clear,
then smoothly returns toward the racing line.  The reference is only a
candidate seed; the unchanged SQP dynamics and all hard proofs retain final
authority.

The ahead disjunct is required: holding the lateral branch through the full
horizon makes a valid completed pass collide with a later wall interval, while
returning the reference without an ahead row makes the obstacle constraint
contradict that return.

## Isolation

An optional forced transition exists only in the shadow snapshot and dynamic
refinement request.  Its default is absent and reproduces the production
producer byte-for-byte.  Candidate fingerprinting includes the forced stage,
but legacy snapshots with no forced stage retain their frozen fingerprint.

The comparison reports every lattice member rather than silently selecting a
failed local optimum.  Any accepted ManeuverBundle is evidence that candidate
generation, not persistent Mission lifecycle or physical infeasibility, was
the blocker.

## Non-scope

- No production authority promotion.
- No partial-escape production change.
- No solver, clearance or configuration change.
- No offline multi-SQP/nonlinear feasibility D unless every C member fails.
