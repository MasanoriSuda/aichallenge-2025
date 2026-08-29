# Requirements: dynamic-wait prefix owner removal

## Frozen baseline

- Baseline commit: `e864829e`
- Dynamic run: `output/20260829-235457`, Domain 1
- Representative episodes: 1 and 9 near waypoint 115

## Observed causal sequence

1. A complete canonical ShiftOut reference exists.
2. The legacy receding-horizon physical check rejects its own rollout and is
   correctly demoted to `canonical-reference-only`.
3. The legacy Mission enters `FollowPrepare` to wait for a replacement.
4. Its optional legacy forward prefix is unavailable.
5. The controller mutates the tactical phase to `Recovery` solely because the
   prefix could not be published.
6. On the same and following decisions, the certified Store still proves and
   publishes the prior seven-state ShiftOut artifact with normal authority.

The phase therefore says Recovery while the only actual production authority
is ShiftOut. Later intent handoff and stale-artifact failures are symptoms of
this split ownership.

## Root cause

`DynamicMissionWaitAction::Hold` is not honored as a hold. A downstream legacy
prefix publication failure creates a second authority decision and changes the
Mission phase to Recovery before canonical current-world admission runs.

## Constraints

- Do not add a resume rule, lease, grace, timeout, retry or fallback.
- Do not change solver settings, clearance, weights or proof tolerances.
- A failed prefix must not gain normal authority.
- Independent hard faults must retain their existing Recovery/Stop behavior.
- Canonical current-world proof remains the only normal command authority.

## Definition of done

- A soft dynamic-wait Hold with no legacy prefix keeps `FollowPrepare` and
  Mission identity unchanged.
- The same-cycle canonical admission may retain a proved ShiftOut/Pass; if no
  proof exists, Emergency Stop remains fail-closed.
- Hard-fault `DynamicMissionWaitAction::Recovery` is unchanged.
- The old prefix-failure-to-Recovery production edge is deleted.
- Focused tests, source contracts, package tests and build pass.
- Dynamic acceptance observes no
  `dynamic Mission wait has no wall-feasible lateral authority` Recovery edge.
