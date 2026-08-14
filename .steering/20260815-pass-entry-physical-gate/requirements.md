# Requirements

## Purpose

The `20260815-061346` `make dev2` run reached Pass in 6 of 9 overtake Missions and
completed one clean `ShiftOut -> Pass -> Return -> Idle`.  The remaining failure
boundary moved to Pass execution.  Two failed Missions published a runtime wall
preplan warning with a predicted 0.35 s wall time-to-contact, still entered or
continued Pass, then failed physical revalidation and reported physical wall
contact.

## Required behavior

- Do not enter Pass while the measured/predicted runtime wall warning band is occupied.
- While waiting for a fresh physically valid path, stop adding lateral displacement and
  keep a wall-validated current-side prefix instead of executing the stale DP prefix.
- Do not hand longitudinal control back to ordinary Follow solely because of this soft
  wall warning when the current and predicted vehicle footprints remain separated.
- Bound the hold by the existing Pass-horizon time and distance limits.  On expiry,
  invalidate the stale Mission and reselect; physical wall/contact faults retain their
  existing Recovery priority.
- A repeated SafeSeparation request must not reset its start time/distance or emit an
  `entered` log every control cycle.

## Constraints

- Preserve physical wall contact, hard wall margin, emergency front-risk, solver recovery,
  target continuity and forbidden-waypoint hard gates.
- Preserve target ID and pass-side ownership while the bounded hold remains valid.
- Do not reduce configured wall or vehicle clearance.
- Do not change ROS topics, message types, services, launch entry points or result schemas.
- Do not modify generated `output/` files or the user's `aichallenge/result-summary.json`.

## Definition of done

- A deterministic core resolver covers hold, release and bounded reselect decisions.
- ShiftOut cannot transition to Pass under an active soft runtime wall warning.
- The hold publishes a physically validated current-lateral path and exposes a diagnostic.
- SafeSeparation entry is idempotent while already active.
- Unit tests and the package build pass.
