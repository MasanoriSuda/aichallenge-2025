# Design: independent nonlinear feasibility oracle

## Solver boundary

The existing dense arm is still one affine QP built around one current
linearization.  The independent oracle instead optimizes the 60 piecewise
constant controls and rebuilds all seven states by exact nonlinear rollout.
CasADi/IPOPT owns this offline nonlinear problem; OSQP matrices, scaling and
workspace state are not reused.

A nonnegative scalar slack is added uniformly to every retained physical
margin and minimized before a tiny control-distance regularizer.  This gives a
typed infeasibility measure instead of a binary optimizer message.  A result
is physically feasible only when the recomputed numerical margins and slack
are both within the fixed audit tolerance.

## Deterministic starts

The oracle tries the recorded controls, input reference, their midpoint and a
fixed-seed bounded family.  Each start solves the same NLP.  Failed starts do
not mutate or warm-start a production solver.

## Certification

The oracle writes a complete state/control primal.  The existing
`mpcc_architecture_compare --external-primal-physical-nonlinear-oracle`
command rebuilds the execution artifact and applies the canonical exact proof
chain.  The Python margin result alone never creates a ManeuverBundle.

Cruise exposed a separate comparison blind spot: the generic A--D tool
rejected every stateless arm as `unsupported-intent` before constructing a
candidate.  The audit-only normal-avoidance entry point therefore rebuilds
explicit positive- and negative-side candidates for Cruise/Follow while
preserving the neutral execution intent.  It has no production caller and
both candidates use the same SQP and exact proof chain as the captured arm.

## Exit classification

- nonlinear feasible and C++ exact proof accepts: single-SQP/model defect;
- nonlinear feasible but C++ proof rejects: oracle/certificate mismatch;
- all deterministic starts retain material slack: physical infeasibility is
  not proven, but this candidate family is unresolved and must not be tuned;
- affine infeasible while nonlinear exact proof accepts: affine trust-region
  or candidate-generation defect.
