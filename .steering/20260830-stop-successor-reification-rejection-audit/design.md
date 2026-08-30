# Design

## Observation boundary

`StopSuccessorResult` already contains the exact nonlinear trajectory and the
dense actuation provenance.  The bundle adapter is therefore the earliest
boundary that knows why an accepted physical Stop cannot be represented as an
immutable seven-state execution artifact.

Add a diagnostic-only subtype to the adapter result.  It is populated at the
existing reject sites and printed by the existing production decision log.
It does not alter the aggregate reason, plan construction, Store, publisher or
Emergency ownership.

## Classification

- exact trajectory/sample shape
- invalid initial lateral bounds
- discontinuous command interval index
- command changed inside one serialized interval
- non-finite/non-positive dense sample
- non-positive aggregated duration
- first command shorter than the publisher interval
- command endpoint progress regression

The last category records progress delta in metres and the sealed distance
tolerance.  This is deliberately separate from virtual progress speed so a
possible unit mismatch can be proven rather than guessed.
