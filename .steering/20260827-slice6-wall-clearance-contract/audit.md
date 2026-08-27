# Audit

## Status

Implementation and dynamic acceptance complete.

## Root cause classification

- Root cause: the canonical problem collapsed or dropped the distinction
  between physical and required planning clearance.
- Contributor: ShiftOut moves the vehicle close enough to expose the missing
  clearance.
- Detection gap: physical snapshot construction hard-coded zero, while
  pre-entry problem construction did not own a physical value at all.
- Mask: downstream wall guards reported wall/Recovery symptoms after the
  contract had already diverged upstream.
- Recovery behavior: Emergency and Overtake Recovery safely removed normal
  authority after the mismatch became visible.

## Hypothesis history

1. Initial hypothesis: all consumers should use the advertised required
   clearance.  Static tests passed.
2. Refutation: `output/20260827-182448` produced persistent Track/Cruise
   wall-refined QP maximum-iteration failures.
3. Revised hypothesis: preserve the existing contract semantics—physical
   proof uses physical clearance; planning uses required clearance.
4. First confirmation: `output/20260827-183852` restored Track/Cruise, but
   exposed `canonical physical wall clearance unavailable` for pre-entry.
5. Final repair: resolve both values before the active-Mission gate.

## Dynamic evidence

`output/20260827-184821` (`make dev2`):

- both domains launched and moved;
- startup wall-refined QP failure count: 0;
- `canonical physical wall clearance unavailable`: 0;
- ShiftOut pre-entry shadow evaluations: 39;
- complete `solver/wall/target accepted` pre-entry results: 3;
- `Idle -> ShiftOut`: observed;
- wall-induced Recovery: 0 during the captured run.

The run also exposed a separate downstream defect: ShiftOut can be paused by
`DynamicMissionWait` or `SafetyBrake` before Pass.  That is not repaired in
this Slice and must be investigated from its own evidence boundary.

## Verification

- `make autoware-build`: passed, 25 packages.
- Focused physical-wall tests: 11/11 passed.
- Focused wall-contract test: 1/1 passed.
- Source authority/contract tests: 63/63 passed.
- Full `multi_purpose_mpc_ros` package test: 1,945 tests, 0 errors,
  0 failures, 0 skipped.

## Invariant after repair

```text
one WallClearanceContract producer
  |- required_clearance_m -> planning/admission bounds
  `- physical_clearance_m -> QP refinement -> fresh proof -> retained proof
```

Neither branch can silently substitute the other.

## Rollback

Baseline commit: `86b2028`.
