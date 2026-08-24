# Design

## Root producer

`resolve_shadow_warm_start()` proves rolling physical stage overlap but drops
the overlap offset.  `solve_extended_progress_problem()` later calls
`shift_mpc_warm_start()` whose current contract always advances one stage.

This separates compatibility from alignment and permits a warm artifact with
the right schema but the wrong physical stage lineage.

## Repair

1. Return the exact overlap offset in `ShadowWarmStartResolution`.
2. Generalize `shift_mpc_warm_start()` to an explicit stage advance.
3. Apply the same advance to state, input, equality dual, box dual,
   curvature-rate dual and every trailing stage block.
4. Thread the resolved advance through each canonical solver context.
5. Report the applied advance with existing warm/reset provenance.

Zero advance copies the certified artifact.  Positive advance drops exactly
that many leading stages and repeats the terminal stage for the uncovered tail.
An advance outside the available horizon is rejected.

## Deleted assumption

The implicit `one solver call == one physical horizon stage` assumption is
removed.  No replacement fallback or configuration is added.

## Scope

This Slice changes warm-start alignment only.  QP formulation, costs, bounds,
physical certificates, selection and command authority remain unchanged.

## Rollback

Rollback commit: `66a76c9`.
