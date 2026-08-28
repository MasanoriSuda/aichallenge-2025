# Design: Native A/B failure classification

## Evidence sequence

```text
current HEAD build
  -> one bounded dev2 observation
  -> first native Pass/Return schema-v2 failure snapshot
  -> immutable replay validation
  -> A persistent candidate
  -> B-left / B-right stateless candidates
  -> unchanged seven-state SQP
  -> common exact wall/opponent/successor proof
  -> architecture classification
```

The existing recorder is observation-only.  A runtime failure is never
induced by changing parameters or authority.  If no relevant failure occurs,
the Slice remains inconclusive and records the minimum additional evidence
needed.

## Upper-log comparison boundary

`.steering/ano` is used only to compare externally visible operating style,
such as continuous command cadence, prolonged stops and recovery frequency.
It cannot supply the exact current-world matrices, wall grid, warm start or
physical certificate and therefore cannot be combined with the native A/B
snapshot.

## Stop condition

Stop and reassess before implementation if A/B does not discriminate the
failure.  Candidate C, offline D, current literature, reference GitHub MPCC
implementations and the upper log are then consulted as architecture evidence;
production remains frozen until that audit selects a bounded next Slice.

