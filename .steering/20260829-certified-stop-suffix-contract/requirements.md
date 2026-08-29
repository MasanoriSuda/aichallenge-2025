# Certified stop suffix contract

## Objective

Prevent a retained seven-state MPCC artifact from keeping production authority
when only the current control stage is physically clear and no certified stop
suffix or certified successor exists.

## Frozen evidence

- Run: `output/20260829-104728/d1/autoware.log`
- Accepted artifact: sequence 709 / source decision 1334
- At decision 1354 the artifact was already retained with
  `last_wall=current-stage-prefix` while commanding about 3.8 m/s.
- At decision 1394 the next stage became current and was rejected as
  `continuation-wall-blocked`; the vehicle was already 0.009 m from the front
  wall and Recovery could not prevent the incident.
- Decision 1434's `initial-state-outside-bounds` is downstream evidence after
  the vehicle had entered the wall envelope; it is not the initiating defect.

## Constraints

- Do not tune wall clearance, solver tolerance, timeout, lease, or vehicle
  limits.
- Do not add another fallback.
- Keep one canonical seven-state normal-control authority.
- A partial current-stage proof may remain diagnostic, but it may not cross
  the production authority boundary until an exact stop suffix is present.

## Root-cause statement

The retained revalidator treated a clear current stage as sufficient authority
even when the later suffix was known to hit a wall. The published command was
allowed to accelerate through that stage without a physically certified way to
stop before the known obstruction. Replanning did not complete before the
unsafe stage became current.
