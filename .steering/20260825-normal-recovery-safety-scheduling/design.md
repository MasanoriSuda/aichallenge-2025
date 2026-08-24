# Design

## Causal chain

```text
ordinary Cruise is clearly moving
  -> Recovery adapter performs wall classification + footprint sampling
  -> detector later returns VehicleMoving
  -> no Recovery action can use the safety result
  -> MPCC tail + redundant Recovery work exceeds the 25 ms callback period
```

The defect is scheduling and responsibility order, not a wall margin or solver
parameter.

## Repair

Introduce one pure eligibility contract:

```text
Recovery safety required :=
  supervisor state is not Normal
  OR
  (ego is within the detector moving-speed boundary
   AND forward intent is present)
```

Non-finite inputs return `true`, preserving fail-closed behavior. The adapter
uses this single result for both the preliminary wall classification and the
full `evaluate_recovery_safety()` call. It still builds one `CoreInput` and
calls `StuckRecoveryCore::update()` on every cycle, so detector observation
reset, supervisor time and ordinary Recovery telemetry remain causal.

This does not add a cache, timeout, cadence or fallback. It removes work whose
result is provably unobservable to the Normal-state supervisor on that cycle.

## Verification

- Pure unit tests cover Normal moving, Normal stopped with/without forward
  intent, active Recovery, and invalid inputs.
- A source contract confirms both wall classification and full safety use the
  same eligibility owner.
- Full package tests and `make autoware-build` must pass.
- dev2 must preserve Stuck detector updates while reducing ordinary Cruise
  Recovery timing; active Recovery behavior is `NOT EXERCISED` unless the run
  naturally enters it.
