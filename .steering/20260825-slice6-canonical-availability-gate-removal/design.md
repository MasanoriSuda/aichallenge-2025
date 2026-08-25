# Design

## Root cause

The three switches were introduced while a legacy MPC owner still existed.  They selected whether
normal control used progress/extended MPCC or fell through to the old formulation.  Slice 6 has
already deleted that fallthrough, but the switches still guard canonical lifecycle construction,
metadata preparation and eligibility.  The downstream dispatch therefore fails closed to
Emergency when a switch is false; there is no longer a legitimate alternative owner.

## Repair

1. Construct the canonical solver lifecycles regardless of runtime YAML.
2. Reduce progress activation to two execution facts: Overtake execution or Dynamic Escape.
   Track/Cruise, Follow and Rejoin continue to request their canonical metadata through intent-
   specific eligibility.
3. Delete configuration-based eligibility fields and rejection reasons.  Eligibility retains only
   current-world facts: intent, concurrent live execution, tactical snapshot, coherent front
   observation, execution context and lateral bounds.
4. Treat the five-state extended dynamics as the canonical formulation, not an optional feature.
5. Delete the three YAML keys and loaders.  The optional dual-branch tactical worker remains
   independently configurable because disabling it does not select a second normal controller.

## Expected behavior

The checked-in runtime configuration already sets all three deleted switches to `true`, so the
reachable normal control graph is unchanged.  The change removes only the ability to create an
ownerless configuration and the dead migration vocabulary around it.

## Rollback

Rollback this Slice as one commit if a focused contract, package test or build fails.  Do not
restore the switches as a fallback; restore the prior accepted commit and investigate the failed
canonical invariant.
