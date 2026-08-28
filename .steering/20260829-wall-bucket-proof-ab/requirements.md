# Requirements

## Objective

Determine whether the frozen ShiftOut failure is caused by redundant hard
heading/lag trust buckets in physical wall refinement, rather than by the
physical wall, opponent or vehicle dynamics.

## Frozen production behavior

- Do not change production authority or the normal seven-state solve.
- Do not change solver tolerances, iteration limits, wall clearance,
  opponent clearance, timeout, lease, fallback or Mission lifecycle.
- Keep lateral/progress wall rows and all actuator/dynamics constraints.
- Keep the final exact wall, dynamic-obstacle and terminal-successor proofs
  unchanged.

## Audit comparison

For the same immutable current-world snapshot, evaluate two observation-only
variants:

- J: omit only physical-refinement heading bucket boxes;
- K: omit only physical-refinement lag bucket boxes.

These variants may construct a `ManeuverBundle` only after the unchanged exact
proofs accept it. They have no store, mailbox, command or publisher path.

## Exit classification

- J or K solves and exact proofs accept: redundant bucket/formulation defect.
- J or K solves but exact proof rejects: bucket is hiding a model/certificate
  mismatch; do not promote the relaxed formulation.
- Both remain infeasible: proceed to a genuinely elastic/multi-SQP or
  nonlinear feasibility oracle.

## Deletion milestone

Delete the audit entry points after the architecture decision is recorded, or
promote exactly one proven formulation while deleting the replaced hard-bucket
path in the same slice.
