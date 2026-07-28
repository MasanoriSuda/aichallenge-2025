# Requirements

## Purpose

Make a committed overtake usable on the narrow simulation course by separating the lateral
clearance required for full Pass acceleration from the physical body-overlap boundary.

## Evidence

In `output/20260728-090011/d1/autoware.log`:

- the locked-target front cap released at lateral separation `1.51 m` and target distance
  `4.03 m`;
- `desired_v` then increased to `11.11 m/s`;
- approximately 3.6 seconds later the same committed Pass transitioned to `SafetyBrake` at
  target distance `1.78 m`;
- the Pass line was immediately discarded even though the lateral separation was oscillating
  around the configured `1.50 m` release boundary.

The configured candidate corridor and execution wall requirements also disagree:

- candidate center corridor: `1.50 + 0.72 + 0.10 = 2.32 m`;
- execution footprint corridor: `1.50 + 0.725 + 0.20 = 2.425 m`.

## Required behavior

- Keep the aggressive `0.10 m` overtake candidate gap.
- Align committed-line wall clearance with that candidate by using `0.10 m`.
- Require the full `1.50 m` inflated separation for full Pass acceleration.
- In a latched Pass, when separation is between the `1.45 m` combined body width and the
  `1.50 m` inflated release threshold, retain Pass and reapply the limited closing-speed cap.
- Below `1.45 m`, retain the existing front-risk and SafetyBrake protection.
- Do not weaken protection for non-locked vehicles, ShiftOut, or an unlatched Pass.

## Constraints

- This is for the competition simulator; no real-vehicle validation is claimed.
- ROS 2 topics, services, message types, and evaluation schemas remain unchanged.
- Actual wall contact, physical execution infeasibility, and existing emergency bounds retain
  priority.

## Acceptance criteria

- A latched Pass at `1.49 m` excludes only the locked target from generic centerline braking but
  does not release the full-speed cap.
- The same target at `1.44 m` remains subject to generic front-risk braking.
- A target at `1.50 m` can release the Pass speed cap.
- Unit tests and `make autoware-build` pass.
