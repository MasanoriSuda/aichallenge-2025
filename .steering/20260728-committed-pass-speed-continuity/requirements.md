# Requirements

## Purpose

Prevent a committed overtake from crawling beside a stopped or very slow locked target after
physical lateral clearance has already been established.

## Observed behavior

- In `output/20260728-003305`, P1 reached `Pass` and released the locked-target front cap.
- The cap was reapplied only because the lateral execution horizon became constrained, even
  though current target separation remained above the reapply threshold.
- After the locked target stopped, the MPC continued following a trajectory reference near
  `0.5 m/s`, extending `Pass` and increasing exposure to later wall/recovery conditions.

## Constraints

- Do not weaken physical wall contact, target-overlap, front-risk, EmergencyBrake, curvature,
  acceleration, or other hard velocity limits.
- A later follow-up may permit initial release only after full physical lateral clearance.
- Preserve ROS 2 topics, services, message types, launch entry points, and evaluation schemas.
- Keep the change configurable and limited to the participant controller.

## Acceptance criteria

- A previously released cap remains released during `Pass` when current lateral separation is
  still above the reapply threshold and the generated execution path remains physically feasible.
- The cap is reapplied if lateral separation falls below the hysteresis threshold, the target is
  unavailable, or the physical path becomes infeasible.
- A configurable reference-only speed floor is applied for a committed, laterally clear `Pass`
  around a stopped/very-slow locked target.
- The speed floor never exceeds the already computed MPC hard speed bounds.
- Unit tests cover constrained-horizon retention and committed-pass speed-floor gating.
