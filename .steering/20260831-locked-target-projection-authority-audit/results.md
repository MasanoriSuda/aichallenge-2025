# Results

## Static verification

- `make autoware-build`: passed for `multi_purpose_mpc_ros`.
- Package tests: 59 suites passed.
- `colcon test-result --verbose`: 2291 tests, 0 errors, 0 failures,
  0 skipped.
- `git diff --check`: passed.

The tests prove that the bounded tactical projection remains the preferred
source, a previously identified active locked target can use the continuity
projection, and continuity cannot create a new identity or bypass a position
jump.

## Dynamic verification

Run: `output/20260831-175431/d1/autoware.log`

- `ShiftOut -> Pass`: 2
- `Pass -> Return`: 2
- `Return -> Idle`: 2
- `locked target stale or lost`: 0
- `continuity_projection=1`: 0

Both observed overtake episodes completed their entire phase chain.  The
frozen false target-loss failure did not recur.  This run did not cross the
exact bounded-projection seam while a target was locked, so it does not by
itself exercise the continuity source; that boundary is covered by the frozen
MCAP evidence and the focused static tests.

## Residual boundary

After both successful overtake episodes, a distinct Track/Cruise production
failure appeared.  Revalidation repeatedly rejected the retained artifact as
`steering-unreachable`, normal authority became an emergency Stop, and stuck
recovery subsequently activated.  This occurred with no active overtake
target and is not repaired in this Slice.

The next Slice must freeze the first `steering-unreachable` transition and
audit physical steering at observation/control origin, expected stage-zero
steering, rate bounds, and the producer-to-publisher time join.  It must not be
treated as evidence against locked-target projection continuity and must not
be patched with a steering tolerance, grace period, or recovery rule.

## Classification

The original frozen failure is a projection/classification lifecycle defect:
raw observation and a current certified artifact existed, while the tactical
horizon seam was incorrectly interpreted as target loss.  It was neither raw
V2X loss nor physical infeasibility.
