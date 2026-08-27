# Design

## Root cause

The phase lifecycle compared measured `e_y` only with the frozen Mission's
original `goal_ey`.  The Mission was admitted at one course geometry, but the
vehicle remained in ShiftOut while the wall corridor evolved.  At wp157--160
the selected side had already gained physical lateral separation, the planned
7 m ShiftOut and 25 m total Mission length had both been exceeded, yet the
fixed 2.20 m goal was no longer reachable.  Reference completeness then kept
the stale Mission alive until the canonical proof withdrew and Emergency
braking/lateral momentum reached the wall.

The wall abort is therefore downstream.  The upstream defect is that
`ShiftOut -> Pass` encoded a path-sample goal as the semantic phase boundary.

## Repair

Introduce a typed ShiftOut completion resolution with two valid completion
proofs after the planned longitudinal shift distance:

1. the planned pass-side lateral goal is reached; or
2. the locked target is continuous and the current course-frame relative
   lateral measurement proves physical center separation on the selected
   homotopy.

The second proof uses the existing physical vehicle separation contract.  It
does not use generic 2-D footprint separation, because a target several metres
ahead is longitudinally non-overlapping before any ShiftOut occurs.

After this semantic boundary, the existing Pass-entry dynamic and physical
horizon gates retain authority.  If the current world cannot support Pass,
the existing typed DynamicWait/reselection path runs; this change does not
make an unsafe Pass executable.

## Observability

The completion resolver returns a reason enum.  The phase transition log says
whether the boundary was reached by the planned goal or by selected-side
physical separation, so a later failure can be attributed to phase lifecycle
or Pass admission rather than inferred from scattered debug fields.
