# Design: Rate-resolved model/proof unification

## Root cause

`linearize_temporal_frenet()` used one coarse stage map: lateral and lag were
advanced from the stage-start heading for about 0.15 s. The physical adapter
replayed the same control at at most 0.01 s and updated heading and response at
every substep. Repeated SQP therefore converged against a different nonlinear
map than the final certificate checked.

The exit classification is **model/certificate mismatch**: the QP solve
succeeds, exact proof fails, and the same frozen problem is feasible under the
certificate dynamics.

## Change

1. Add one canonical seven-state nonlinear stage evaluator in
   `mpcc_rate_resolved`.
2. Compose it with at most 10 ms midpoint substeps.
3. Numerically linearize that exact composed map at the SQP reference. This
   gives a consistent affine tangent without maintaining a second analytic
   discretization.
4. Make the physical adapter's dense replay call the same evaluator for each
   substep.
5. Replace tests tied to the removed coarse Euler formula with tests that
   assert reference-point exactness and adapter/model equivalence.

## Non-goals

- No change to costs, bounds, clearances, solver tolerances, or correction
  counts.
- No production authority change.
- No new fallback or special-case acceptance.

