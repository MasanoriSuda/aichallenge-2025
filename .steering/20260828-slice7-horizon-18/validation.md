# Validation

## Static Gate

- `make autoware-build`: passed, 25 packages.
- `test_single_authority_source_contract`: passed.

## Dynamic runs

First run: `output/20260828-050046`.

- Two `ShiftOut -> Pass -> Return -> Idle` episodes completed.
- No Overtake Recovery or actual wall-margin violation occurred.
- Callback overrun rate was 0.410% (21/5119 cycles), versus 1.785%
  (102/5713) in the 20-stage baseline.
- Mean MPCC one-second-window time was 2.016 ms versus 3.810 ms; maximum
  MPCC cycle time was 41.865 ms versus 56.310 ms.

Those results initially supported the hypothesis, but both successful
episodes selected the same `side=+1`.  A fresh simulator/Autoware restart was
therefore required before acceptance.

Independent run: `output/20260828-050412`.

The shortened future proof failed in two separate episodes:

```text
ShiftOut -> FollowPrepare
  reason=dynamic Mission wait: live overtake corridor unavailable
FollowPrepare -> Recovery
  reason=static wall clearance margin infeasible

ShiftOut -> Recovery
  reason=actual footprint wall margin violated
```

## Decision

Rejected.  The favorable first run was not reproducible.  Eighteen stages
reduce the computation tail, but—like the previously rejected 16 stages—can
remove the wall-feasible terminal successor needed to keep the committed pass
safe as geometry changes.  Local and cloud configurations are restored to
`N=20`.

The result narrows the next experiment: preserve the 20-stage proof and reduce
how often a fresh full solve is requested, while revalidating the last actually
published certified artifact on intervening control cycles.  Do not shorten
the horizon further or compensate with clearance/solver tolerance changes.
