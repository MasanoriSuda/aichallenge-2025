# Design

## Root cause

The migration elevated the asynchronous seven-state MPCC to production
authority but left the old `optimize_live_overtake_line_horizon()` invocation
inside `init_problem()`. Its output no longer owned actuation when the
canonical reference contract was complete, yet it still ran inside the 40 Hz
control callback and could consume roughly a full callback budget by itself.

The failure chain was:

1. OvertakeLine built a complete reference/corridor.
2. The obsolete legacy optimiser re-solved that reference synchronously.
3. Its infeasible result was correctly demoted to reference-only.
4. The canonical seven-state solve independently succeeded.
5. The redundant synchronous work delayed publication and the next snapshot.
6. An emergency Stop command entered the physical successor between solves.
7. The retained artifact no longer joined the measured steering state and its
   reconstructed Stop suffix intersected the wall.

The wall/terminal proof is therefore exposing a scheduling/integration defect;
weakening that proof would only hide it.

## Ownership after the change

- OvertakeLine tactical layer: target, homotopy, phase, goal profile and
  stage-wise physical corridor.
- Seven-state MPCC: continuous lateral/longitudinal optimisation and normal
  actuation authority.
- Current-world proof: immutable wall/dynamic certificate and retained
  revalidation.
- Emergency supervisor: explicit Stop authority only when no certified normal
  successor joins.

`evaluate_overtake_line_horizon()` remains because it is the reference and
corridor builder. `optimize_live_overtake_line_horizon()` is no longer invoked
from production problem assembly because it is a second optimiser over the
same decision variables.

## Non-goals

- This Slice does not relax Stop suffix certification.
- This Slice does not claim that all callback overruns are eliminated;
  recovery safety accounting is measured separately after removing this known
  duplicate owner.
- This Slice does not tune racing performance.

## Dynamic acceptance

- Run: `output/20260830-023852/d1/autoware.log` (`make dev2`, about 110 s).
- `OvertakeLine runtime ownership`: 0 occurrences.
- `OvertakeLine runtime ownership ... receding=1`: 0 occurrences, compared
  with 1 occurrence and 19.171 ms of receding work in the frozen failure.
- Six ShiftOut episodes reached certified seven-state production authority;
  the removed legacy optimiser was not needed to construct or admit them.
- In ordinary Cruise, representative callback windows returned to roughly
  3--5 ms average with zero overruns in those windows.

The run did not establish Pass quality: none of the six ShiftOut episodes
reached Pass. Five ended through `DynamicMissionWait` because the Pass-entry
physical gate had no valid current-side prefix or remained unresolved; one
ended after the locked target became stale/lost. Emergency and
`steering-unreachable` events also remain outside this removed call edge.
Those are independent reference/admission and retained-join defects and must
not be hidden by reintroducing the duplicate optimiser or relaxing proof.
