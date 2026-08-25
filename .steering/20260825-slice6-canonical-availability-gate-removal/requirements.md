# Requirements

## Objective

Remove the expired runtime migration gates that can disable the sole canonical normal MPCC owner
after the legacy, three-state and direct normal authorities have been physically deleted.

## Earliest violated invariant

Normal intent ownership is selected by control intent.  It must not depend on the former
`progress_contouring_mpcc_enabled`, `progress_contouring_mpcc_overtake_only`, or
`progress_contouring_extended_dynamics_enabled` migration switches.  With no legacy normal owner
remaining, a false or absent switch creates an ownerless normal cycle rather than a valid fallback.

## Scope

- Remove the three obsolete runtime configuration keys and their C++ storage/loading.
- Make canonical solver lifecycle construction unconditional.
- Resolve progress activation from the current execution intent only.
- Remove migration-switch rejection reasons from Track/Cruise, Follow, Rejoin and Overtake
  canonical eligibility.
- Preserve dual-branch selection as an optional tactical producer; it is not a normal-command
  formulation switch.

## Non-scope

- No weight, clearance, margin, timeout, solver tolerance or cadence changes.
- No new fallback, retry, hold, grace or feature flag.
- No rename-only cleanup of the remaining historical `shadow` identifiers.
- No Emergency or Recovery behavior change.

## Definition of done

- A failure-first source contract proves that none of the three migration switches can be
  represented in production or runtime configuration.
- Track/Cruise, Follow, Overtake and Rejoin canonical eligibility cannot be disabled by a runtime
  formulation switch.
- Focused tests, package tests and the canonical Docker build pass.
- The migration ledger records the deletion and names any remaining Slice 6 work.
