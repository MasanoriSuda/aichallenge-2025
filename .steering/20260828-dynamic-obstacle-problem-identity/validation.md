# Validation

## Static

- `make autoware-build`: passed, 25 packages.
- `colcon test --packages-select multi_purpose_mpc_ros`: passed, 52/52
  CTest targets.
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros
  --verbose`: 2,036 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.

## Dynamic

Bounded two-vehicle run: `output/20260828-230302`.

- `Idle -> ShiftOut`: observed once for the primary Overtake episode.
- canonical ShiftOut certified/retained publication: observed.
- `physical obstacle world does not match problem identity`: 0.
- `dynamic obstacle refinement has no matching problem identity`: 0.
- Cruise and Follow continued to produce certified seven-state normal
  candidates with an active stay-behind obstacle constraint.

## Separate residual failure

The primary episode later transitioned:

```text
ShiftOut -> FollowPrepare
reason=dynamic Mission wait: live overtake corridor unavailable
```

This occurred after the dedicated constraint identity was accepted and after
certified ShiftOut publication.  It is therefore a later Mission/corridor
lifecycle failure, not evidence to weaken or revert this Slice.
