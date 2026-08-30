# Requirements

## Objective

Freeze production authority and determine whether the terminal Stop suffix
which certifies a bounded normal publication interval can actually join the
next control-origin observation.

The failure under investigation is the first ShiftOut episode in
`output/20260830-133656`.  The normal publisher-interval prefix remains clear,
but retained authority is lost when a newly rebuilt terminal Stop becomes
wall-infeasible.  The production adapter currently discards the earlier
certified terminal trajectory and Emergency Stop rebuilds another base-track
command.

## Constraints

- Do not change production authority or command selection in this slice.
- Do not add Mission resume rules, leases, grace periods, timeouts, fallbacks,
  solver tolerances, or clearance changes.
- Keep the exact seven-state trajectory and its causal actuation samples under
  one immutable identity.
- Promote observation evidence only after the associated canonical normal
  command is verified at the publication boundary.
- Treat an unmatched state/time/identity join as evidence, never as permission
  to execute the successor.
- Preserve ROS 2 and evaluation interfaces.

## Definition of Done

- Terminal Stop construction exposes the exact input applied to every physical
  integration sample and the end of the publisher interval.
- Retained proof and production authority preserve that evidence without
  changing authority resolution.
- A successfully serialized canonical normal command records one immutable
  pending Stop successor observation.
- The next normal control origin emits one structured join result containing
  time, pose, velocity, and steering deltas.
- Unit/contract tests reject incomplete or misaligned successor evidence.
- Build and focused tests pass.

