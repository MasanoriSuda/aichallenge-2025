# Results: published progress-to-path alignment

## Root cause

The published Overtake consumer advanced an exact trajectory with measured
course progress, then used that value as an offset on the trajectory's nominal
path-distance axis.  Those coordinates diverge on curved and laterally moving
execution.

The first attempted correction—resampling directly on course progress—was
falsified by `output/20260830-165202`: certified progress may plateau during
lateral motion and is therefore not a valid strictly increasing interpolation
axis.  This was an important contract distinction, not a parameter issue.

The final implementation projects measured course progress onto the certified
path coordinate.  If more than one path point shares that progress, the exact
publication cursor chooses the causally matching point.  Lateral resampling and
remaining coverage then stay entirely on the monotonic path axis.

## Static verification

- `git diff --check`: passed.
- Source-contract test: 87 passed.
- `make autoware-build`: 25 packages passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 2209 tests passed,
  0 errors and 0 failures.
- Unit coverage includes deliberately unequal path/progress coordinates and a
  progress plateau selected by publication time.

## Dynamic verification

Run: `output/20260830-170541/d1/autoware.log`

- `published trajectory resampling failed`: 0 occurrences.
- `course-progress-projection-unavailable`: 0 occurrences.
- Published ShiftOut alignment became active from a current-world Bundle.
- A cursor-exhausted Bundle was rejected explicitly and the next exact source
  aligned successfully.
- The first ShiftOut lasted 4.67 seconds and ended through the independent
  external-Recovery path.  No Pass-progress or wall-margin conclusion is drawn
  from this short run.

## Scope boundary

No Mission transition, authority rule, lease, grace period, timeout, fallback,
solver tolerance, clearance or speed parameter changed.  The external-Recovery
episode and remaining Pass/Return quality stay as separate follow-up evidence.
