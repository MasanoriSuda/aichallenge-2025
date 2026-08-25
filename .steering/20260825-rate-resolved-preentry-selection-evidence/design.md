# Design

## Root cause addressed

The previous shadow solved and physically proved a six-state left/right
candidate, but reduced the result to booleans and terminal metrics. It discarded
both the solver objective and the immutable CertifiedPlan. Therefore it could
not answer either promotion question:

1. would six-state choose the same executable side as the current Gate A;
2. can the exact selected six-state proof survive the async handoff.

Deleting five-state without those answers would replace evidence with an
assumption.

## Evidence flow

```text
one immutable tactical snapshot
  -> left and right prospective six-state solves
       -> exact execution artifact
       -> exact static-wall proof
       -> current target-tube proof
       -> immutable CertifiedPlan + objective/terminal/reserve metrics
  -> six-state-only branch selection (observation)
  -> compare with production five-state selection
  -> compact telemetry only
```

The production five-state selection continues to own Mission admission in this
Slice. The observation selection is not read by candidate selection,
`apply_mpcc_entry_execution_contract`, the FSM, retained production store, or
the final publisher.

## Promotion criterion

A later atomic Slice may replace Gate A only after dynamic evidence shows:

- the six-state artifact is complete for the encountered pre-entry intents;
- selected-side disagreement is understood rather than hidden;
- the immutable artifact can be current-world revalidated at adoption;
- failure retains Track/Follow and cannot fall through to legacy authority.

That later Slice must delete the tactical five-state solver lifecycle and
five-state pre-entry canonical-plan representation in the same change.
