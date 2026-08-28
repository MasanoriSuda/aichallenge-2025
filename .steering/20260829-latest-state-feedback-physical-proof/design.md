# Design

## Shared proof path

The retained evaluator keeps one proof pipeline after actuator reachability:

```text
current state
  -> nonlinear continuation
  -> measured-to-control wall proof
  -> continuation wall proof
  -> timed dynamic-obstacle proof
  -> Follow gap proof when applicable
```

Normally reachable candidates use the immutable prepared steering.  An
unreachable observation arm uses the analytical feedback steering from
`mpcc_latest_state_feedback` as the continuation initial command.  All later
proof code is shared.

## Observation-only boundary

The observation arm records its downstream proof result but restores the
production result to `SteeringUnreachable` and never constructs `Proof`.
Consequently `mpcc_rate_resolved_production_adapter` cannot see or publish the
corrected command.

## Why this is a refactor, not another fallback

There is no second wall or dynamic checker and no bypass.  A single local
completion boundary maps downstream failures either to the normal production
result or to an observation-only classification.  Promotion is a later atomic
change that must create a newly certified feedback artifact and delete the old
elapsed-suffix-only adoption path.
