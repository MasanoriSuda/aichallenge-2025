# Requirements

## Objective

Physically remove the retired three-state normal-formulation representations and the lossy
five-state-to-three-state conversion helper after the normal solver fallthrough was deleted.

## Repaired invariant

The execution contract may represent only formulations which still have an intentional runtime
role. A deleted normal solver must not remain reconnectable through an enum, schema switch or
conversion API.

## Earliest violation

`LegacySpatialMpc3State` and `ProgressContouring3State` have no production producer. Their only
production consumers are string/schema switches. `convert_extended_solution_to_legacy()` has one
declaration, one definition and zero production call sites; only its dedicated tests invoke it.

## Scope

- Delete the two retired formulation enum values and string/schema branches.
- Delete the lossy five-state-to-three-state conversion declaration and implementation.
- Replace rejection tests which manufacture a retired formulation with the live, explicitly
  noncanonical `SolverDerivedBypass` formulation.
- Delete tests dedicated solely to the removed conversion.
- Add a source-level deletion contract which prevents reconnection.

## Explicit non-scope

- Do not change the five-state or six-state MPCC mathematical formulation.
- Do not change Rejoin production authority.
- Do not remove `SolverDerivedBypass`, Emergency or Recovery contracts.
- Do not tune solver, wall, obstacle, horizon, weight, rate or timeout parameters.
- Do not rename historical documentation or generic local variables merely containing the word
  `legacy` when they are not a runtime authority.

## Acceptance

- The failure-first deletion contract fails before implementation and passes afterwards.
- `make autoware-build` passes.
- All package tests pass with `BUILD_TESTING=ON`.
- Static search finds no retired formulation or conversion symbol in production source.
- No user-owned generated result is staged.

## Rollback

Revert the single commit produced by this Slice.
