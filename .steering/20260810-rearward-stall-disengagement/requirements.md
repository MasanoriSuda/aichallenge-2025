# Requirements

## Objective

Prevent a committed Pass from remaining on a speed-losing lateral line until
the local SafeSeparation distance limit, then entering a lateral Recovery near
the wall.

## Observed failure

In `output/20260810-203315/d1/autoware.log`, the locked target moved from
approximately `-1.03 m` to `-0.60 m` while SafeSeparation forward completion
remained active.  The best-distance progress was stale for `4.20 s`; the local
distance limit then selected Recovery, which was followed by wall contact and
solver failures.

## Constraints

- Preserve physical wall, emergency-brake, solver-recovery, target-continuity,
  current-body-overlap and execution-corridor guards.
- Do not switch across the target after tactical no-return.
- Do not merge to the reference line before normal rear clearance.
- Keep the behavior bounded and configurable.
- Preserve ROS 2 topic, message, launch and evaluation interfaces.

## Acceptance criteria

- Detect a rearward target that is catching ego after longitudinal progress
  has become stale.
- Keep the already separated pass side and let the target clear forward,
  instead of immediately starting lateral Recovery.
- Re-enter the existing dynamic Mission revalidation only after the target is
  continuously clear ahead.
- Add deterministic unit tests for admission, hold, release and timeout.
- Pass package tests and `make autoware-build`.
