# Requirements

## Goal

Prevent a nearly completed Pass from discarding its Mission merely because a
new centerward wall-escape prefix is temporarily unavailable, when a connected
and physically validated target-bound execution prefix is already running.

## Observed failure

In `output/20260819-102554`, episode 12 reached Pass with the locked target at
approximately `-0.60 m`, current body footprints separated, and an active
target-bound physical execution hold. A runtime wall warning then failed to
produce a centerward escape prefix and immediately invalidated the Mission.
Contact, wall fault, Recovery, and solver fallback followed.

## Constraints

- Physical wall contact, hard wall faults, unavailable wall samples, emergency
  front risk, solver Recovery, and overtake-forbidden sections keep priority.
- Do not extend an unvalidated nominal path.
- Do not change ROS topics, messages, services, launch entry points, or result
  schemas.
- Preserve unrelated user changes, including `aichallenge/result-summary.json`.

## Definition of Done

- Runtime wall preplanning can retain the current side when a committed,
  rearward Pass has a connected target-bound execution prefix.
- Without that evidence, a failed evaluated contraction still exits the
  current Mission as before.
- Hard wall faults are never overridden.
- Unit tests and the package build pass.
