# Evidence

## Frozen failure

- Run: `output/20260830-110056/d1/autoware.log`
- Sequence 3460 / decision 4065 / Cruise
- Sequence 3541 / decision 4146 / Cruise
- Both normal-avoidance homotopies rejected before solve.
- Common reason: `target course projection unavailable at stage 0`.

## Causal chain

1. The canonical problem captures a valid target tube and ego progress bounds.
2. The physical snapshot builds course knots from ego progress bounds only.
3. Stateless normal avoidance rebuilds the target from current-world Cartesian
   state to avoid retaining stale Mission geometry.
4. The peer lies beyond the ego-only knot window at stage zero.
5. Projection fails identically for both sides, so the candidate population is
   empty and fresh normal authority cannot be produced.

## Validation

### Refuted hypothesis

An ego-plus-target course-window resolver was implemented and passed:

- focused stateless maneuver tests;
- full package CTest, 55/55;
- `make autoware-build`, 25 packages.

Run `output/20260830-111337/d1/autoware.log` nevertheless reproduced the same
failure six times, including sequences 3198, 3279 and 3341.  The extension was
removed before commit.

### Next observation

The course projector now reports whether the failure is invalid input, invalid
knot, non-monotone progress, a zero-length segment, course-frame sampling,
non-finite projection, or absence of an eligible segment.  The next run will
select the actual structural correction.

### Static correlation

- Failure location: `wp_id=338..339`.
- `traj_mincurv_org.csv` waypoint 348:
  `(89656.8036175, 43128.8719252)`.
- waypoint 0: the same coordinate.
- The circular horizon therefore contains a monotone-progress, zero-world-
  length `348 -> 0` segment.
- Source behavior before the fix: return an invalid projection immediately on
  that segment, discarding all adjacent usable segments.

## Root-fix validation

- Run: `output/20260830-112453/d1/autoware.log`
- More than 6,000 control decisions and multiple laps observed.
- Frozen signature
  `target course projection unavailable at stage 0`: **0**.
- At waypoint 338, the controller produced an active current-world
  `dynamic-escape` path instead of losing both candidates during projection.
- Three ShiftOut episodes were entered; two reached Pass in the captured run.

The run exposed separate downstream failures:

- two Pass entries moved to `DynamicMissionWait` because the physical gate had
  no valid current-side prefix;
- a later ShiftOut ended with `actual footprint wall margin violated`.

Those are not course-projection failures and are intentionally deferred to a
new Slice rather than folded into this correction.

## Verification

- `git diff --check`: passed.
- focused `test_mpcc_stateless_maneuver`: 24/24 passed.
- full package CTest: 55/55 passed.
- `make autoware-build`: 25 packages passed.
- `make dev2`: completed and stopped cleanly.
