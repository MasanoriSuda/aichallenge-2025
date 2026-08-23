# Audit

## Evidence ledger

| Observation | Classification | Evidence |
|---|---|---|
| Entry certificate is accepted before both failed ShiftOut episodes | upstream fact | `output/20260824-055552/d1/autoware.log` |
| Physical branch proof converts the five-state primal to legacy first | root-cause evidence | `mpc_controller_cpp.cpp`, extended branch evaluation |
| Stored certificate contains only path distance and lateral samples | root-cause evidence | `ExtendedBranchEvaluation` and Mission certificate schema |
| Entry revalidation reconstructs a lateral-only trajectory | root-cause evidence | `revalidate_overtake_entry_execution_certificate` |
| Live swept wall guard rejects after entry | downstream symptom / useful guard | episodes 1 and 2 in the same run |
| Reference wall clearance remains high when live proof rejects | detection mask | decision/debug log reports reference rather than exact solved artifact reserve |

## Current classification

- Confirmed root cause of the admission/proof semantic split: loss of
  five-state physical trajectory provenance before physical certification.
- Confirmed independent contributor after that repair: stage-wise wall rows can
  admit an exact solution whose continuous swept connection intersects a wall.
- Refuted as the observed live failure's dominant cause: progress linearization
  drift. The first exact failure had only `0.213105 m` progress delta and
  `-0.0351523 m` lag, but the segment entering stage 15 still intersected the
  wall.
- Repaired detection gap: entry and live proof now carry lateral, lag, heading,
  velocity and solved progress; live rejection reports the exact stage,
  waypoint and progress delta.
- Correct guard: swept physical wall rejection must remain fail-closed.

## Causal chain

Before this Slice, the extended solver's exact state sequence was flattened to
the legacy three-state representation before branch wall proof. That erased lag,
heading and solved progress. Mission admission then stored and revalidated only
lateral samples, while live swept validation reconstructed a different pose
sequence. Therefore an accepted entry and a later wall rejection did not refer
to one reproducible physical artifact.

The repair constructs an immutable exact physical trajectory immediately from
the normalized five-state primal. Branch selection, Mission admission and live
execution now use that artifact before legacy command adaptation. Legacy
conversion remains temporarily downstream as a command/prediction adapter and
cannot certify a wall path.

## Dynamic evidence

`output/20260824-063046/d1/autoware.log` provides the first closed-loop proof:

- waypoint 170: `Idle -> ShiftOut` is admitted with a complete physical
  certificate;
- decision 2393: the live five-state trajectory passes the exact swept proof and
  is published as `extended-mpcc-solved`;
- decision 2525, 3.35 seconds later: a newly solved five-state horizon is
  rejected at stage 15 / waypoint 201 with `ey=0.377657 m`,
  `lag=-0.0351523 m`, `epsi=0.00745163 rad` and
  `progress_delta=0.213105 m`;
- no lateral-only certificate is admitted, and the rejection is attributable to
  one exact artifact rather than a reconstructed legacy projection.

The remaining swept collision is not evidence that this Slice failed. It is a
newly isolated upstream formulation gap: pointwise stage wall bounds do not yet
prove the continuous swept segment between states. That must be handled in a
separate bounded Slice rather than by weakening this guard or adding Recovery
policy.

## Existing patch relationship

- `convert_extended_solution_to_legacy()` remains only because the publisher
  path has not yet been fully migrated to canonical five-state actuation.
- The executed swept wall guard remains authoritative and exposed the remaining
  continuous-geometry defect correctly.
- Recovery, cooldown and alternate-side logic were not changed; they are
  downstream reactions, not causes of this Slice's semantic split.

## Next bounded question

Determine whether the stage-15 failure is caused by chord/swept geometry omitted
from QP construction or by course-frame interpolation disagreement. Add a
failure-first continuous-segment constraint/proof test before changing the
formulation. Do not tune wall margin, solver tolerance or Recovery behavior.
