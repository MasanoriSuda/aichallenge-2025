# Requirements

## Objective

Promote the dynamically accepted current-world Stop successor from the last
actually published normal artifact without creating a second normal command
owner.  Replace the direct ordinary authority-loss-to-external-Emergency edge
only when the exact same-intent Stop trajectory is physically certified in the
current world.

## Baseline and evidence

- Baseline: `4ecf3628 feat(mpcc): prove current-world stop successor`
- Run: `output/20260830-180130`, decision 4060
- Ordinary Pass authority was unavailable while the shadow successor was
  accepted by exact model, wall and all-peer checks.

## Constraints

- No Mission resume rule, lease, grace, timeout, fallback, solver tolerance,
  wall clearance or configuration parameter is added.
- The Stop successor must become a new immutable seven-state certified plan;
  a controller-local speed/steering override is forbidden.
- The command must cross the existing canonical normal publisher and only
  then enter the publication ledger.
- Preserve target, intent, intent generation and homotopy identity.
- Rebuild state, path and physical proof from the current control origin.
- Intent mismatch, invalid current world, wall/dynamic blockage, invalid
  physical rollout and missing source remain external-Emergency failures.
- `Return` remains fail-closed unless its terminal semantic contract is
  actually reached; this Slice may not relabel a stopping point as a Return.

## Exit criteria

- Accepted non-Return successor produces a valid certified plan and canonical
  normal authority with maximum braking.
- Rejected or semantically unsupported successors cannot publish or mutate the
  certified-plan Store.
- The old direct external-Emergency edge is unreachable for the accepted
  successor case, while every other case preserves it.
- Focused tests, full package tests and build pass.
- Dynamic `make dev2` evidence confirms normal Stop-successor publication at a
  real authority-loss decision and no one-cycle authority regression.
