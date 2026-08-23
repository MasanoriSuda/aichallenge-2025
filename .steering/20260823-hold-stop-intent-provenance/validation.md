# Validation

## Root-cause evidence

Historical run `output/20260822-031809`, Domain 1, decisions 4534 and 4909
recorded:

```text
phase=FollowPrepare
action=dynamic-wait
lateral_owner=dynamic-wait-prefix
longitudinal_owner=pass-floor
speed_window=6.00/inf/6.00
command=6.06m/s
```

The old private action switch mapped every `DynamicWait` to canonical `Hold`.
That mapping confused a rolling ShiftOut/Pass replan prefix and a held lateral
Mission path with a longitudinal zero-progress wait. A future zero-progress
Hold formulation would therefore have converted valid execution states into a
stop.

Historical run `output/20260820-094153`, Domain 1, also recorded a non-prefix
DynamicWait with `line=1`, no speed owner and `ego=5.48 m/s`. This proves that
the old reason text `dynamic-wait-hold` names lateral ownership only; it is not
evidence of stationary longitudinal intent.

The same run separately recorded `SafetyBrake` as the actual hard longitudinal
owner with `-3.00 m/s2`; Stop is therefore kept as an explicit supervisor
intent rather than inferred from DynamicWait.

## Static verification

- `make autoware-build`: passed, 25 packages.
- `test_overtake_execution_orchestrator --gtest_color=no`: passed, 64/64.
- `git diff --check`: passed.

The focused tests prove:

- lateral-hold and rolling DynamicWait preserve ShiftOut or Pass origin;
- missing lateral authority, mission identity or supported origin fails closed;
- SafetyBrake resolves to Stop;
- pre-race and active-race Cruise resolve to Track and Cruise respectively;
- final control telemetry includes the canonical intent joined to the published
  command.

## Dynamic verification

Run: `output/20260823-132619`

The final decision log exposed the semantic join without changing command
selection:

```text
decision=582  action=cruise canonical_intent=track/track-before-race-session
decision=877  action=follow canonical_intent=follow/resolved-action
decision=962  action=cruise canonical_intent=cruise/cruise-during-race-session
```

The corresponding execution contracts remained their existing production
owners: certified five-state MPCC for Track/Cruise and legacy normal bypass for
Follow. No Hold/Stop or overtake candidate was promoted.

The short run did not enter DynamicWait. Positive runtime proof with the new
trace remains a later dynamic gate; historical rolling and lateral-hold
evidence plus pure tests are sufficient only for the provenance repair.

## Authority and diff audit

- No publisher connection changed.
- No path, velocity, acceleration or steering selection changed.
- No parameter, feature flag, fallback, timeout or lease was added.
- The controller-private action switch was replaced by one pure canonical
  resolver shared by the MPCC problem context and both authority/final traces.
- Hold/Stop QP bounds and production authority remain intentionally absent.
- No existing DynamicWait is labeled as longitudinal Hold.

## Remaining risk and next gate

The next Hold Slice is blocked until a true longitudinal-hold producer is
identified. It must not infer zero progress from DynamicWait. Stop remains an
explicit SafetyBrake supervisor until a separately approved authority Slice
proves equivalent hard stopping behavior.
