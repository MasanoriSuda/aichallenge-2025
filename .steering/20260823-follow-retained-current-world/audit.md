# Follow retained current-world audit

## Findings before implementation

1. `canonical_retained_revalidation` already provides the correct immutable plan, exact cursor,
   current intent/target identity, stage identity and proof fingerprint boundary.
2. `canonical_retained_world_revalidation` currently manufactures obstacle-clear evidence only for
   an explicitly current empty V2X world. It returns `DynamicObstaclePresent` when any target exists.
3. Follow fresh execution already proves the physical hard gap using `theta + e_lag`, but its
   `FollowShadowCycleResult` does not retain the extracted plan and there is no Follow plan store.
4. The Follow `MpccProblemContext` already seals target ID, target observation generation, stage
   geometry, intent generation and the Follow-specific bounds/cost schema.
5. The current `FollowLongitudinalContract` supplies a fresh stage-wise target forecast and hard
   gap. It is the correct current observation boundary for retained revalidation; recreating target
   motion from old plan data would be stale by construction.

## Root cause

The remaining availability gap is a missing lifecycle/proof integration, not a Follow tuning
problem: a fresh canonical plan is complete but is discarded at the end of the cycle, while the only
retained current-world implementation rejects the dynamic target that defines Follow.

## Scope decision

Implement a typed target-tube proof and retained shadow lifecycle. Do not alter final command
authority, legacy/scalar Follow ownership, solver settings, or configuration.

