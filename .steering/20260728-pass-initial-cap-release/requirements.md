# Requirements

## Purpose

Allow P1 to create longitudinal progress after it has already established physical lateral
separation in `Pass`, including when a feasible lateral horizon is still reference-clamped by a
wall or lateral-acceleration limit.

## Evidence

In `output/20260728-084735/d1/autoware.log`:

- `ShiftOut -> Pass` occurred four times, but `Pass -> Return` occurred zero times.
- Only one Pass released the locked-target front-speed cap.
- One representative Pass latched physical lateral exclusion at `2.34 m`, but retained
  `cap_release=0` because the execution horizon was wall-limited.
- Its reference then followed target speed plus `0.5 m/s`, falling from `4.48` to `0.78 m/s`.

## Constraints

- Initial constrained-horizon release requires current lateral separation at the full release
  threshold, not only the lower reapply hysteresis threshold.
- Release is limited to `Pass` after front-overlap exclusion has latched.
- Physical path infeasibility, actual wall contact, target loss, incomplete lateral shift, and
  existing front-risk/Emergency hard limits retain priority.
- The change must not modify ROS 2 interfaces or evaluation schemas.

## Acceptance criteria

- A constrained but physically feasible `Pass` can release the cap for the first time when
  current lateral separation is clear.
- A constrained initial release is rejected when only the lower reapply threshold is satisfied.
- Existing release hysteresis remains valid after initial release.
- Unit tests, `make autoware-build`, and package tests pass.

