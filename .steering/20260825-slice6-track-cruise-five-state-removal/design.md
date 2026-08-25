# Design

## Before

```text
Track/Cruise runtime
  -> six-state production owner

compiled but unreachable legacy
  -> TrackCruise mode
  -> five-state synchronous solver context
  -> five-state plan store
  -> retained empty-world proof
  -> Track/Cruise telemetry

Rejoin runtime
  -> same mode-switched five-state evaluator
```

## After

```text
Track/Cruise runtime
  -> six-state production owner

Rejoin runtime
  -> explicit Rejoin-only five-state evaluator
  -> Rejoin-only solver context and plan store
```

The change removes the dead Track/Cruise branch at its shared-mode boundary.
It does not replace it with a compatibility flag or alias. The common
five-state solver implementation remains available to Follow, Overtake and
Rejoin because those authority migrations are separate vertical Slices.

## Deletion set

- `CanonicalNormalShadowMode::TrackCruise` and the mode enum;
- `track_cruise_shadow_solver_context_`;
- `track_cruise_shadow_plan_store_`;
- Track/Cruise five-state warm-start identity and context epoch;
- Track/Cruise five-state retained evaluator;
- Track/Cruise five-state telemetry window/status and recorder;
- reset logic that clears the retired five-state plan store;
- dead request-draft construction inside the shared five-state evaluator.

The six-state fields still carrying historical `shadow` names are not renamed
in this deletion Slice because they are live transport/proof objects. Naming
cleanup will be a separate mechanical Slice after authority deletion, so it
cannot obscure this source-level proof.
