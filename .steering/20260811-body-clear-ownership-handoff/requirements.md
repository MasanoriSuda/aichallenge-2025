# Requirements

## Goal

Prevent an admitted overtake Mission from falling back to Follow solely at the
`ShiftOut -> Pass` phase boundary before its predicted body-clear deadline.

## Observed problem

- The latest run admits kinematically feasible Missions, but Behavior repeatedly
  changes from Overtake to Follow while OvertakeLine still executes ShiftOut or
  an unlatched early Pass.
- `body_clear_deadline_feasible` is frozen as a boolean and remains true even
  after the predicted hard-distance arrival time has passed.
- The front-danger suppression exception applies to ShiftOut only, so changing
  the phase label to Pass can immediately revoke longitudinal ownership.

## Constraints

- Do not suppress confirmed current body overlap, wall/path faults, target
  discontinuity, emergency conditions after the deadline, or solver recovery.
- Do not change ROS 2 topics, messages, launch contracts, evaluation schema, or
  acceleration limits.
- Preserve the existing 15-second Mission total budget and all downstream wall,
  lateral-acceleration, corridor, and Recovery guards.
- Do not modify `aichallenge/result-summary.json`; it is user/run output.

## Acceptance criteria

- A frozen, feasible Mission has a finite absolute body-clear handoff expiry.
- The handoff is active only before expiry and while current overlap is not
  confirmed.
- A committed early Pass can retain Behavior ownership through that bounded
  handoff without requiring a legacy lateral/front-cap latch.
- Front-danger suppression can cross `ShiftOut -> Pass` only under the same
  bounded handoff and current separated-body condition.
- Unit tests cover active, expired, overlap, and hard-guard cases.

