# Findings

## Static validation

- `make autoware-build`: passed (25 packages).
- `colcon test --packages-select multi_purpose_mpc_ros`: all 59 tests passed.
- Aggregate total reported by `colcon test-result`: 2298 tests, zero errors,
  zero failures.

## Dynamic validation

Run: `output/20260831-060156` (`make dev3`).

Domain 2 exercised the previously disconnected path:

```text
accepted async tactical result (target=d3, side=-1, tactical sequence=1)
  -> Gate A draft source=async-preentry-homotopy
  -> publication worker-submitted
  -> seven-state solve=solved
  -> exact physical wall=accepted
  -> dynamic proof=clear
  -> current-world join=accepted
  -> Gate A proposal selected
  -> OvertakeLine Idle -> ShiftOut
```

The causal solve took about 216 ms, but it was isolated from the 40 Hz
publisher and the result remained joinable against the current world.  The
committed certificate contained 483 exact trajectory stages with 0.20 m
required wall clearance.

This is the first dynamic evidence that the async tactical owner can enter the
canonical ShiftOut producer without importing its old trajectory or authority.

## Next failure boundary

The admitted episode did not complete.  About 5.9 seconds after entry it
transitioned `ShiftOut -> Recovery` with `locked target stale or lost`.
The entry lifecycle defect is therefore fixed, while target continuity during
the committed execution is the next independent causal boundary to audit.

No solver tolerance, wall clearance, target clearance, lease, timeout,
fallback or production publisher ownership was changed in this Slice.
