# Design

## Initial hypotheses

1. **Mission lifecycle defect**: an entry-time gap/corridor producer is still
   allowed to invalidate an already published receding ShiftOut artifact.
2. **Candidate-generation defect**: the frozen goal is valid, but the live
   gap planner searches only a disconnected or wrong-side interval and calls
   the whole maneuver blocked.
3. **Physical infeasibility**: at target distance about 2 m, neither side has
   enough footprint separation even though the static-wall corridor is wide.
4. **Model/certificate mismatch**: the seven-state QP uses stay-behind rows
   while the phase supervisor assumes a pass-side constraint has already been
   achieved.

## Investigation order

1. Trace `overtake_execution_corridor_blocked` from planner output to phase
   transition.
2. Reconstruct the same decision's target, wall, body, side and artifact
   identities.
3. Check whether the certified artifact was still publication-valid when the
   live signal invalidated the Mission.
4. Only after classification, compare correction alternatives.  If the
   existing abstractions cannot distinguish them, add observation-only typed
   evidence before changing authority.

## Frozen producer/consumer trace

The first hypothesis that the failure was caused merely by entering ShiftOut
before `/awsim/state=Start` is rejected.  `docs/spec/mpc-integration.md`
defines `Ready -> Start` as one planning session when start-grid rollout is
enabled, and the vehicle was physically progressing during this interval.

The blocking signal is produced by the generic `V2XGapPlanner` even while the
explicit OvertakeLine owns the path.  In that ownership mode its bounds are
not applied to the controller, but `planner_output.active &&
!planner_output.feasible` still becomes `overtake_execution_corridor_blocked`
after the active gap-loss hold.  The ShiftOut continuity consumer then
invalidates the Mission generation and enters DynamicMissionWait.

At the failing boundary the selected side was rejected by the generic planner
as `forced-side-empty` / `pass-side gap unavailable`.  The opposite side was
reported wider, but the longitudinal no-return latch was active.  Meanwhile
the canonical seven-state artifact remained a stay-behind trajectory: its
dynamic contract had stay-behind rows and no pass-side rows.  Static wall and
current footprint checks were clear, but that artifact did not certify future
completion on the selected side.  Therefore neither blindly ignoring the
generic planner nor treating its entry topology as the canonical physical
certificate is justified.

## Required comparison evidence

The run `output/20260828-230302` predates the interaction-snapshot schema-v2
writer and contains only exact-QP schema-v1 artifacts.  It cannot support the
required physical A/B/C comparison without inventing obstacle provenance.
The next action is therefore evidence capture on the current HEAD, with
production authority unchanged.  The first replay-ready ShiftOut failure at
or before the live-corridor invalidation will be compared through the existing
architecture escape-hatch harness before any production correction.

## Same-world classification

The current-HEAD run `output/20260828-231959` produced schema-v2 ShiftOut
snapshots without changing production authority.  Two snapshots were used:

- sequence 8, dynamic-obstacle refinement rejection near the start-grid
  rollout;
- sequence 2970, post-refinement rejection while the vehicle was moving at
  course progress 56.73 m.

For both immutable worlds, the persistent arm A and the current bounded
production population failed.  Stateless direct-side B either failed the SQP
or solved but failed the nonlinear opponent proof.  The exhaustive audit arm
found multiple physically certified right-side diagonal schedules.  For the
moving snapshot, schedules such as stage 11 -> 19 and stage 13 -> 19 were
accepted while the production population still rejected both sides.

This classifies the failure as **candidate-generation defect**.  It is not
evidence for changing solver tolerances, wall clearances, Mission leases or
authority.  `build_bounded_candidates()` currently contains only:

1. direct side; and
2. the earliest possible physical diagonal (first obstacle stage -> +2).

The accepted audit trajectories delay the diagonal transition and complete
it at the terminal stage.  The production set therefore structurally omits a
feasible temporal homotopy.

## Structural correction

Keep the population bounded and add one normalized late-transition candidate:

- start at two thirds of the stage span from the first obstacle stage to the
  terminal input stage;
- reach the full pass side at the terminal input stage;
- use the existing physical-diagonal constraint construction;
- pass through the unchanged seven-state solver, nonlinear wall/opponent
  proof, terminal-successor check and certified store.

The two-thirds knot is a topology sample, not a relaxation of physics.  It is
derived from the current horizon and first active obstacle stage, so no
track-specific distance, timeout or clearance is introduced.  The earliest
candidate remains for fast feasible cases; the late candidate covers the
distinct wait-then-shift homotopy demonstrated by both frozen snapshots.

## Offline acceptance after correction

The unchanged comparison harness was rebuilt and rerun on both schema-v2
snapshots after adding the bounded late candidate:

| Snapshot | Production left | Production right |
|---|---|---|
| sequence 8 | proof rejected | accepted, late diagonal 13 -> 19 |
| sequence 2970 | proof rejected | accepted, late diagonal 13 -> 19 |

The accepted arms retained the original wall/opponent nonlinear proof and
terminal-successor gates.  No clearance, solver tolerance, lifecycle rule or
publisher authority changed.  This is the expected result for a
candidate-generation correction: the same immutable physical problem and
same seven-state refinement become publishable only because the previously
missing temporal homotopy is now present.
