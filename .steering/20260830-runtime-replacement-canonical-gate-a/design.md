# Design

## Root cause

There are two tactical producer paths:

1. new-entry dual-branch selection;
2. active same/cross-side runtime replacement.

The execution layer consumes path 2 during an active ShiftOut or Pass, but the
causal seven-state Gate A worker consumes path 1 in every phase. When path 1 is
invalid or selects a different side, no Gate A proposal is emitted even though
path 2 contains the candidate the execution layer is actively trying to adopt.
The old Mission is then retained until its wall certificate expires.

## Ownership rule

Introduce one pure `RateResolvedGateATacticalInput` resolver:

- Idle: pre-entry selected candidate owns the Gate A input.
- Active ShiftOut/Pass/FollowPrepare:
  - same-side runtime candidate owns when it is the execution layer's first
    candidate;
  - otherwise cross-side runtime candidate owns;
  - pre-entry selection is not a fallback.
- Invalid or mismatched candidates yield no input and therefore fail closed.

The causal worker still rebuilds the seven-state problem from the current
serialized predecessor and reruns exact wall and dynamic proof. The resolver
changes only which tactical intent is certified, not the proof or publisher.

## Atomic replacement

The resulting certified plan is bound back to the selected Mission candidate
as its exact physical execution trajectory. Runtime replacement continues to
check target, side, prospective generation, intent, and physical contracts
before mutating Mission state. This preserves the existing atomic handoff.

## Removed edge

Active execution no longer depends on
`rate_resolved_preentry_branch_selection` or
`rate_resolved_preentry_selected_mission_hint`. Those fields remain the sole
new-entry producer.

## Non-goals

- Changing candidate ranking or homotopy cost.
- Adding a second runtime worker.
- Changing no-return policy.
- Stop suffix redesign.
- Parameter tuning.
