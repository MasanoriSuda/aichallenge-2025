# Results: Rough disjunction lattice candidate C

## Frozen evidence

- Production baseline: `5c81bd38`
- Run: `output/20260828-094214`, Domain 1
- Snapshot: `000000001566-pass-wall-refinement-solve-rejected`
- Interaction fingerprint: `7246006054995400977`
- Production authority/configuration changes: none

## Comparison

The same current-world snapshot, seven-state SQP, solver policy, wall model,
opponent model, clearances and terminal successor contract were used for every
arm.

| Arm | Result |
|---|---|
| Persistent A | solver rejected |
| Stateless left B | solved, exact dynamic proof rejected |
| Stateless right B | solver rejected |
| Rough lattice C | 0 accepted / 420 evaluated; all solver rejected |

Candidate C enumerated both homotopies and every valid pair
`first_side_stage < first_ahead_stage <= N` for `N=20`.  Its constraint rows
use only the complete physical disjuncts:

1. stay behind;
2. full lateral separation;
3. longitudinally ahead after rear-clear.

No partial-escape row is present in C.  Adding the missing ahead/return phase
removed the representational contradiction of holding a side branch through
the whole horizon, but did not produce a feasible first SQP subproblem.

## Classification

This result does **not** prove physical infeasibility.  It only falsifies the
tested rough branch-timing lattice under the current single-SQP refinement.

- `A fails, B succeeds` is false because B has no physically certified bundle.
- `A/B fail, C succeeds` is false because all C members fail before exact proof.
- Physical infeasibility remains `Unknown`; no bounded certificate exists.
- The next required arm is D: bounded offline multi-SQP or nonlinear
  feasibility on the same immutable snapshot.

## Root-cause status

- Confirmed detection gap in B: partial-escape rows can satisfy the convex QP
  without satisfying either member of the physical collision disjunction.
- C closes that detection gap for the comparison, but its all-failed result
  cannot distinguish candidate-generation limits from single-SQP limits.
- No production fallback, threshold, clearance, lease, timeout or solver
  tolerance was added.

## Verification

- `test_mpcc_rate_resolved_dynamic_obstacle`: 14/14 passed.
- `test_mpcc_stateless_maneuver`: 9/9 passed.
- `test_mpcc_architecture_snapshot`: 5/5 passed.
- `test_mpcc_architecture_comparison`: 4/4 passed.
- Full package CTest: 52/52 targets passed.
- `make autoware-build`: 25 packages built successfully.
