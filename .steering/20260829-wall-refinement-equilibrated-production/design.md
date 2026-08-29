# Design: wall-refinement equilibrated production owner

## Cause-to-change mapping

The defect is not physical infeasibility and not a Mission lifecycle failure.
Wall-bucket construction creates a numerically distinct KKT class.  The
current explicit physical row normalization plus disabled internal
equilibration rejects feasible wall QPs at the dual termination boundary.
Using the same policy globally is also wrong because it regresses Follow and
does not repair the dynamic-only counterexample.

Therefore solver ownership is partitioned by QP formulation, not selected by
failure outcome:

| QP class | Sole owner |
|---|---|
| Initial / successive linearization | existing row-normalized solver |
| Dynamic-obstacle-only | existing row-normalized solver |
| Wall refinement | row-normalized + internal OSQP equilibration |
| Coupled wall/opponent refinement | row-normalized + internal OSQP equilibration |
| Latest-state feedback | existing row-normalized solver |

The decision is made before `solve()`.  There is no sequence such as
`old solver rejects -> equilibrated solver retries`; hence this does not add a
fallback authority or hide a physical failure.

## Authority and proof boundary

Both solver instances return physical-coordinate primals and duals.  Their
outputs feed the same `verify_external_primal`-equivalent production proof
chain.  The physical wall proof, timed obstacle proof, terminal successor
proof and certified-prefix publication boundary are unchanged.

The shared receding warm start remains a physical artifact, so it may seed the
next immutable QP independent of the selected numerical workspace.  Each
solver keeps its own OSQP workspace cache and is serialized by the existing
`SolverContext` mutex.

## Excluded scope

- Post-refinement physical-dynamic SQP remains on the existing owner until a
  frozen failure identifies its numerical class.  Expanding the policy by
  resemblance alone would repeat the patch-driven failure mode.
- Architecture comparison arms remain observation-only.
- No parameter tuning is part of this Slice.

## Dynamic acceptance finding

The production owner partition removed the observed wall-refinement numerical
failure without increasing median pipeline cost.  The first D1 stop in the
dynamic run was instead:

`Follow retained cursor exhausted`
→ `new Follow state-box QP reached max iterations`
→ `no certified artifact available`
→ `Emergency Stop for 3 s`
→ `Stuck Recovery`.

The same class of stop predates this change.  It is therefore intentionally
excluded from the wall-solver fix and becomes the next architecture audit
target.
