# Requirements

## Objective

Remove the remaining cross-formulation runtime Mission replacement authority.
Every same-side or cross-side replacement during ShiftOut, Pass or
FollowPrepare must be admitted by the causal six-state Mission Gate A before
the frozen Mission can mutate.

## Constraints

- Keep one `VelocitySteeringProgress6State` normal authority.
- Do not add fallback, flag, lease, timeout or parameter tuning.
- Retain the current proven Mission when a causal replacement proof is absent.
- Preserve Emergency and Recovery boundaries.
- Delete the superseded five-state pre-entry/replacement resolver in the same
  Slice.
- Do not stage generated result JSON.

## Exit criteria

- No runtime replacement can mutate Mission from geometry or a five-state
  canonical artifact alone.
- Requested replacement side, target, generation and intent must match the
  exact six-state proposal.
- The proposal's Mission, not a separately reconstructed candidate, is frozen.
- `resolve_overtake_preentry_plan()` and its tests are physically removed.
- Build, package tests and a moving acceptance run pass.
