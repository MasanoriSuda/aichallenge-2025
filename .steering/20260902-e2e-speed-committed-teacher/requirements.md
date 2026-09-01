# Requirements

## Objective

Replace the disproven stateless `precontact_teacher` only in a new diagnostic
mode with a teacher whose avoidance and braking decisions are physically bound
to current wheel speed and whose selected escape side cannot reverse at the
last moment.

## Evidence

- Seed 2031 passed the strict race gate with the historical teacher.
- Unseen seed 2032 finished but received one crash penalty.
- Immediately before that penalty the vehicle was travelling at about
  3.79 m/s.  The historical teacher continued to request `+0.6 m/s2` while the
  frontal return collapsed from 8.59 m to 1.80 m, then reversed lateral intent
  and first requested braking inside the physically recoverable envelope.
- At `-1.0 m/s2`, stopping from 3.79 m/s alone requires about 7.2 m, before
  reaction delay or vehicle clearance is included.

## Constraints

- Do not modify the production default, frozen v11 checkpoint or its SHA.
- Do not change the historical `gap_teacher` or `precontact_teacher` behaviour;
  their existing labels and provenance remain immutable.
- Use the existing wheel-speed topic and final control-command interface.
- Missing or stale speed must fail closed in the new diagnostic mode.
- A failed rollout must not be extracted as a hard teacher label.
- Verify the candidate by offline replay and synthetic unit tests before a new
  closed-loop run.

## Definition of done

- The new mode has a distinct class and runtime identity.
- Speed-dependent stop/preview distances are auditable per decision.
- A committed side is released only after confirmed clear observations.
- An early opposite-side request must persist for multiple scans before it can
  switch authority; one first seen inside the static slow envelope produces
  braking and neutral steering instead of a late cross-side reversal.
- A bilateral side pinch inhibits forward acceleration before the obstacle
  enters the narrow frontal sector.
- Existing controller tests and the ROS package build still pass.
- A new unseen-seed run is admitted only on Finish, zero penalty and zero stall.
