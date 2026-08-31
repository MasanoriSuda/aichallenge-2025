# Design

## Root cause

The persistent Mission timeout was introduced as a tactical liveness policy,
but it is executed inside the production phase owner. It can therefore mutate
the FSM while the canonical Store still owns a valid, publisher-aligned
seven-state artifact. The two authorities are not joined atomically.

Increasing the timeout would only postpone the same race. Adding a handoff
grace would create another lifecycle patch. The consistent repair is to remove
normal authority from the non-physical clock.

## Change

- Continue computing `MissionTotalBudgetResolution` for diagnostics.
- Log Return/Abort observations as `authority=observation-only`.
- Remove all phase, retry-block and retention mutations from this observer.
- Let rear-clear, current-world terminal successor proof, hard fault and
  certified Stop remain the existing state/authority owners.

## Architectural effect

Persistent tactical state is reduced toward encounter identity, selected
homotopy, commit state, and the last published certified artifact. Elapsed
Mission time no longer overrides a receding current-world solution.

