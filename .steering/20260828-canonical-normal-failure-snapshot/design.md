# Design

## Earliest violated observation invariant

Every canonical seven-state normal authority failure must be capturable at the
same numerical boundary that rejected it. The existing recorder instead used
`is_overtake(intent)` as an eligibility gate. As a result, Follow failure
analysis had solver summaries but not the exact cost, constraints, bounds,
scaling, warm start or immutable source context.

## Selected change

Replace the Overtake-only eligibility check with the existing canonical-normal
contract:

```text
canonical_normal_intent_supported(intent)
```

This admits Track, Cruise, Follow, ShiftOut, Pass, Return and Rejoin. Unknown,
Hold and Stop are rejected. Snapshot schema, exact-QP payload, interaction
fingerprint and output deduplication are unchanged.

Exact-QP loading remains valid for every admitted intent. The stronger
`load_recorded_interaction_snapshot()` continues to require a complete replay
world; therefore admitting Follow does not falsely claim that Overtake
ManeuverBundle A--D comparison is applicable.

## Non-scope

- No fix for the observed Follow solve rejection in this Slice.
- No Cruise artifact extension across the transition.
- No retry, grace, timeout, tolerance or clearance adjustment.
- No normal command source or publisher change.
