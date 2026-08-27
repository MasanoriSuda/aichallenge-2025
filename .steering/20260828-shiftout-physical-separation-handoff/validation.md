# Validation

## Failure-first proof

The focused test was rebuilt explicitly so it did not reuse the installed
library or a stale test binary.

```text
V2XOvertakeCoreSpeed.AcceptsSelectedSidePhysicalSeparationAtShiftBoundary
Actual: false
Expected: true
```

The request reproduces the observed boundary: `traveled=32.92 m`,
`required=7.0 m`, `current_ey=1.57 m`, obsolete `goal_ey=2.20 m`, target
relative lateral `-1.59 m`, physical center separation `1.45 m`, side `+1`.

After the implementation, the focused target passes.  Wrong-side separation,
1 cm physical shortfall, invalid observation, and incomplete longitudinal
distance remain rejected.

## Static validation

- `make autoware-build`: passed, 25 packages.
- complete package CTest: 49/49 targets passed.
- `test_v2x_overtake_core`: passed with 801 tests, including the new
  completion contract.
- `test_single_authority_source_contract`: passed; no normal authority or
  legacy path was added.

Two initially reported test issues were stale local test artifacts: the new
architecture-snapshot target had not been built, and an orchestrator binary
predated an enum ABI change.  Rebuilding those exact targets and running the
suite against the current build libraries produced 49/49.  They are not
runtime/product defects from this Slice.

## Dynamic Gate

Run: `output/20260828-044759`, bounded `make dev2`, stopped with `make down`.

Observed episodes in Domain 1:

1. ShiftOut reached the planned goal, Pass-entry physical preflight detected
   excessive lateral acceleration, and the bounded gate selected
   `DynamicMissionWait` rather than Recovery or a wall collision.
2. A later ShiftOut was paused by SafetyBrake; this remains integration-quality
   evidence, not a reason to alter this lifecycle repair.
3. `ShiftOut -> Pass` occurred with a fresh dynamic and physical Pass horizon;
   later SafetyBrake paused the Pass.
4. The new semantic handoff was exercised directly:

```text
Idle -> ShiftOut                         wp=25
ShiftOut -> Pass                         wp=35
reason=shift complete (selected_side_physical_separation)
Pass front-overlap exclusion latched     wp=36
Pass -> Return                           wp=48
Return -> Idle                           wp=60
```

The selected-side handoff occurred about 1.50 s after ShiftOut entry.  The run
contained two `ShiftOut -> Pass` transitions, one `Pass -> Return`, one
`Return -> Idle`, no `ShiftOut/Pass -> Recovery`, and no
`actual footprint wall margin violated` event.

## Conclusion

Accepted.  The frozen-baseline failure is classified as a Mission lifecycle
defect: a path-sample goal incorrectly owned phase completion after the
physical pass homotopy had been acquired.  The repair moves that semantic
boundary upstream and preserves all existing Pass admission proofs.  It does
not establish complete race-quality acceptance; SafetyBrake pauses and
Pass-entry lateral-acceleration infeasibility remain separately typed Slice 7
evidence.
