# Results: proof-consistent dynamic SQP audit

## What was compared

`mpcc_architecture_compare --physical-dynamic-sqp-only` evaluates six
observation-only arms from one immutable snapshot:

- persistent single-SQP and bounded dynamic-SQP;
- production left single-SQP and bounded dynamic-SQP;
- production right single-SQP and bounded dynamic-SQP.

The dynamic-SQP arm rebuilds seven-state dynamics, oriented physical obstacle
supports, and physical wall rows around one common latest primal for three
outer iterations.  It has no Store, mailbox, publisher, or production caller.

## Frozen dynamic-obstacle failure

Snapshot:
`20260829-012053/.../000000001612-shiftout-dynamic-obstacle-refinement-solve-rejected`

- normal production left/direct-side: QP solved, exact dynamic proof rejected
  at `-0.00269008 m`;
- dynamic-SQP left/the same direct-side candidate: all exact proofs accepted;
- normal production right/late-physical-diagonal: accepted.

This proves that a single obstacle convexification can reject a physically
certifiable candidate.  The left candidate was recovered without changing
the world, homotopy, clearance, tolerance, or solver configuration.

## Frozen post-linearization failure and counterexample

Snapshot:
`20260829-012053/.../000000001675-shiftout-post-refinement-linearization-solve-rejected`

- normal production left/direct-side: accepted;
- fixed-count dynamic-SQP left: later iterate became QP-infeasible;
- normal production right/late-physical-diagonal: exact dynamic proof rejected;
- fixed-count dynamic-SQP right: no certified candidate.

The persistent dynamic-SQP arm reduced nonlinear node position error to about
`8e-6 m`, but the resulting branch was physically in collision at a stage
node.  Numerical model convergence alone therefore does not imply tactical or
physical acceptance.

## Classification

The original single-SQP hypothesis is partly confirmed: proof-consistent
successive convexification can recover a candidate that single-SQP loses.
However, unconditional fixed iteration is not a valid production replacement
because it can discard an already-certified solution and enter another local
infeasible basin.

The missing architectural component is an acceptance/merit policy, consistent
with successive-convexification literature:

1. evaluate each new iterate with the unchanged exact proof chain;
2. accept only a certified improvement;
3. retain the last actually certified artifact otherwise;
4. stop when proof/merit converges or the existing bounded audit budget ends.

This is not a timeout or fallback rule.  It is the outer nonlinear solver's
iterate-acceptance contract.  Production authority remains unchanged until
that contract is demonstrated on the frozen corpus.

This direction was checked against current primary implementations and
literature rather than inferred from this repository alone:

- [ct-SCvx](https://github.com/UW-ACL/ct-scvx) explicitly couples successive
  convexification with trust-region/acceptance management;
- [Liniger et al., Optimization-based Autonomous Racing of 1:43 Scale RC Cars](https://arxiv.org/abs/1711.07300)
  supports receding-horizon MPCC rather than retaining one long executable
  Mission trajectory;
- [TUD-AMR MPC Planner](https://github.com/tud-amr/mpc_planner) separates
  candidate/topology generation from continuous MPC refinement.

## Verification

- `test_mpcc_rate_resolved_shadow`: 38/38 passed
- `test_mpcc_architecture_comparison`: 13/13 passed
- `test_mpcc_rate_resolved_dynamic_obstacle`: 20/20 passed
- `test_mpcc_stateless_maneuver`: 17/17 passed

`make autoware-build` completed all 25 packages successfully.  The only
stderr output was the existing setuptools `setup.py install` deprecation
warning.

The comparison test verifies the six-arm audit is separate from production,
and normal `SolverContext::evaluate()` explicitly reports that the audit was
not requested.
