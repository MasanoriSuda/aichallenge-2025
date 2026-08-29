# Requirements

## Objective

Distinguish a physically executable short projected prefix from an exact
ShiftOut-to-Pass successor trajectory.  The former must request successor
replanning and may not authorize Pass by itself.

## Frozen evidence

- Baseline commit: `5b46f11b fix(mpcc): separate terminal stop course geometry`
- Dynamic run: `output/20260830-014758/d1/autoware.log`
- Episode 2 reports `execution_ay=5.87 m/s2` and episode 4 reports
  `execution_ay=4.73 m/s2`, both below the unchanged configured limit of
  `6.0 m/s2`, but both are classified as
  `execution horizon exceeds lateral acceleration limit`.
- Both episodes then leave ShiftOut through DynamicMissionWait without ever
  entering Pass.

## Constraints

- Do not change lateral-acceleration, wall-clearance, solver or timing
  parameters.
- Do not add a lease, grace period, timeout, retry, resume rule or fallback.
- A profile whose accepted maximum lateral acceleration exceeds the limit
  remains unavailable.
- A profile requiring a wall clamp remains unavailable at the Pass-entry gate.
- Production authority and exact wall/opponent proof remain unchanged.

## Definition of Done

- Pass-entry feasibility distinguishes an exact executable successor, a
  projected prefix requiring successor replanning, and a profile that still
  exceeds the physical acceleration limit.
- Failure-first tests cover the 4.73/6.0 projected-prefix, 8.15/6.0 physical
  violation and wall-clamp cases.
- Package build and full package tests pass.
- A `make dev2` run confirms that a projected prefix cannot authorize Pass
  without an exact successor proof.
