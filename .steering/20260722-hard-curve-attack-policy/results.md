# Results

## Verification

- `make autoware-build`: 25 packages finished successfully.
- Full `multi_purpose_mpc_ros` suite: 569 tests, 0 errors, 0 failures.
- The curve-side unit tests prove inside selection at the 3.0 m boundary, outside fallback below
  that boundary, rejection when neither side qualifies, and no switch away from an unavailable
  locked side.
- The intrusion regression proves that selected-side ordering remains authoritative at 2.0 m but
  is ignored at 7.0 m while ego is still behind.

## Runtime observations

- `output/20260722-081557/d3/autoware.log` selected an inside curve entry and later selected an
  outside hard-curve entry. This proved the new side policy, while exposing an entry/continuation
  callback handoff that caused `Overtake -> Follow` before an explicit line was locked.
- `output/20260722-082048/d1/autoware.log` proved both policy outcomes again after the handoff fix,
  but showed the legacy lateral-ordering guard cancelling ShiftOut immediately for targets still
  7-9 m ahead.
- `output/20260722-082809/d1/autoware.log` showed ordering cancellation only when the target reached
  1.99 m, then a subsequent right-side `ShiftOut -> Pass` at waypoint 68 and lateral-clear latch of
  1.37 m at waypoint 83. Far-target ordering no longer cancelled the line.

## Remaining runtime acceptance

The short runs prove policy selection and control-state continuity, not complete physical passes
through every hairpin. A full user-observed dev3 race should confirm one inside and one outside
pass, body/wall clearance, collision count, and whether the 2.0 m near-target ordering threshold
needs further tuning.
