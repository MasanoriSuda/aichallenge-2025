# Results: Offline disjunction continuation candidate D

## Frozen evidence

- Production baseline: `e9323a1b`
- Run: `output/20260828-094214`, Domain 1
- Snapshot: `000000001566-pass-wall-refinement-solve-rejected`
- Interaction fingerprint: `7246006054995400977`
- Production authority/configuration changes: none

## Comparison

The final constraint set, seven-state model, wall and opponent geometry,
clearances, solver policy and terminal-successor proof are unchanged from
A/B/C.  D first tried every exact final disjunction from a cold context.  It
then retried the same schedule with obstacle rows continued through fractions
`0.00, 0.25, 0.50, 0.75, 1.00`; only fraction `1.00` could be accepted.

| Arm | Result |
|---|---|
| Persistent A | solver rejected |
| Stateless left B | solved, exact dynamic proof rejected |
| Stateless right B | solver rejected |
| Rough lattice C | 0 accepted / 420 evaluated |
| Offline continuation D | 0 accepted / 420 evaluated |

All 420 direct-final D solves were rejected before exact proof.  Continuation
then localized the failure by homotopy:

- right: 210/210 failed at fraction `0.00` in the unchanged wall-refined QP;
- left: 210/210 solved fraction `0.00`, then failed while strengthening the
  effective-progress disjunction;
- left failure distribution: 135 after one solved step, 31 after two solved
  steps and 44 after three solved steps;
- no candidate reached and solved the exact fraction `1.00` problem.

Representative left failures were on
`dynamic-obstacle-effective-progress` rows at stages 2 or 3.  This excludes a
simple cold-start explanation: a solved wall-only point was available and was
reused, but the bounded convex continuation could not reach the final exact
disjunction.

## Classification

- `A fails, B succeeds` remains false because B has no certified bundle.
- `A/B fail, C succeeds` is false.
- D did not produce a certified bundle, either directly or by bounded
  continuation.
- Physical feasibility remains **Unknown**.  Failure of OSQP/SQP, including
  every tested continuation path, is not a bounded infeasibility certificate.

The evidence narrows the remaining alternatives to:

1. the tested behind/side/ahead candidate family omits a feasible nonlinear
   trajectory;
2. the single-convex-subproblem representation cannot traverse the required
   nonlinear basin;
3. the frozen world is physically infeasible over this bounded horizon.

The next comparison must therefore be an independent nonlinear or
kinematically bounded feasibility audit.  It must not alter production solver
tolerances, clearances, Mission lifetime or authority.

## Root-cause status

- Mission lifecycle is not sufficient to explain this frozen failure: the
  stateless and continuation arms also fail to form a certified bundle.
- The right homotopy is blocked earlier by the wall-constrained formulation.
- The left homotopy has a wall-only witness but loses convex feasibility as
  the complete obstacle disjunction is imposed.
- Candidate-generation versus nonlinear-optimizer limitation is still not
  separated.

## Verification

- `test_mpcc_architecture_comparison`: 4/4 passed.
- `test_mpcc_rate_resolved_dynamic_obstacle`: 15/15 passed.
- `test_mpcc_stateless_maneuver`: 10/10 passed.
- Frozen D replay: 420/420 evaluated, 0 accepted.
- Full package CTest: 52/52 targets passed.
- `make autoware-build`: 25 packages built successfully.
