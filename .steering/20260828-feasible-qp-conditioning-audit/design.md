# Design

## Audit stages

1. Replay the serialized QP through the unchanged C++ solver, warm and cold.
2. Reconstruct physical `P/q/A/l/u` and reconfirm affine feasibility with
   SciPy HiGHS while ignoring the objective.
3. Reconstruct the exact variable and row transforms used by
   `PersistentOsqpSolver`.
4. Measure matrix coefficient ranges, variable scales, row scales, Hessian
   spectrum and equality/active-row rank.
5. Localize the worst residual row and compare it with near-dependent rows
   having the same physical owner.
6. Decide whether the failure is preconditioning, redundant constraints,
   objective degeneracy or an OSQP implementation limitation.

Offline experiments may change a copied problem to falsify a hypothesis, but
they may not silently become production settings.
