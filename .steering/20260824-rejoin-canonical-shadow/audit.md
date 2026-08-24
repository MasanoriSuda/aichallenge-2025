# Audit

## Causal chain

```text
ShiftOut exact wall-margin failure
  -> OvertakeLine Recovery
  -> orchestrator Action::Recovery
  -> canonical ControlIntent::Rejoin
  -> Rejoin absent from canonical normal domain and all canonical producers
  -> old three-state normal owner publishes
```

## Classification

- Root: incomplete canonical intent migration.
- Trigger: actual wall-margin failure in ShiftOut.
- Mask: legacy Recovery command restores motion and hides the authority discontinuity.
- Not established: whether a five-state Rejoin is dynamically feasible from this state.

## Promotion status

`BLOCKED`. This steering creates observation and proof only. A dynamic Rejoin run is required.

## Dynamic observation

The bounded `make dev2` run at `output/20260824-092036` did not enter line
`Recovery`, so `canonical_intent=rejoin` and the new Rejoin shadow chain were
`NOT EXERCISED`. This is not positive evidence for production promotion.

The run did expose an earlier, independently actionable break in the same
Overtake episode:

```text
Idle -> ShiftOut
  -> exact canonical worker repeatedly reports maximum iterations / pending
  -> no selectable current-world canonical ShiftOut authority
  -> explicit canonical Emergency Stop
  -> zero speed for about three seconds
  -> Stuck Recovery takes authority
  -> external recovery completion resets ShiftOut directly to Idle
```

The episode summary records 5.75 seconds entirely in `ShiftOut`, minimum speed
0.00 m/s and final reason `external recovery completed`. Because Recovery was
completed externally rather than by `OvertakeLinePhase::Recovery`, this run
cannot validate Rejoin.

### Decision

- Keep Rejoin observation isolated and shadow-only.
- Do not add retained Rejoin semantics or production authority.
- Audit the observed async producer/consumer availability break in a separate
  Slice before attempting another Rejoin dynamic Gate.
