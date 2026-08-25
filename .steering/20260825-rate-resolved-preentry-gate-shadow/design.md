# Design

## Current flow

```text
immutable tactical snapshot
  -> left/right candidate Mission
  -> five-state solve
  -> wall + target proof
  -> five-state pre-entry plan
  -> branch selection
  -> Mission mutation
  -> six-state production solve
```

The formulation boundary is therefore still inside Gate A.

## Shadow flow

```text
immutable tactical snapshot + explicit prospective intent
  -> common semantic extended problem
  -> five-state Gate A (production unchanged)
  -> six-state request (shadow)
       -> side-private solver context
       -> exact physical adapter
       -> static-wall certificate
       -> target-tube certificate
       -> metrics only
```

The shadow result is copied only as telemetry. It has no command, Mission
mutation, final publisher or production retained-store connection.

## Intent contract

Pre-entry intent is derived once from the candidate and source phase:

- direct pass or same-side Pass continuation: `Pass`;
- otherwise: `ShiftOut`.

The six-state context must be sealed with that explicit intent. Calling
`current_control_intent()` here would incorrectly fingerprint the still-live
Follow owner.

## Physical contract

Six-state feasibility requires all of:

1. solver result is solved and certified;
2. exact six-state artifact adapts to a complete physical trajectory;
3. swept yawed footprint clears the static wall grid;
4. the same current target tube used by Gate A is laterally excluded across
   the complete horizon.

Target proof is not inferred from wall acceptance and may not be deferred to
the publisher.

## Promotion boundary

This Slice collects evidence only. A later Slice may promote the six-state
artifact only if it achieves sufficient dynamic coverage, and must delete the
five-state Gate A, warm state and canonical-plan representation in the same
change.
