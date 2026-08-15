# Design

## Source synthesis

- **Leading-car logs:** evaluate left/right over roughly 2.4 seconds, continue
  driving on the current feasible solution, and treat the opponent as a dynamic
  obstacle rather than restarting a pass Mission.
- **Pro proposal:** retain the FSM as supervisor and make ShiftOut through
  rear-clear a receding-horizon execution problem with last-feasible fallback.
- **Referenced MPCC:** reuse arc-length progress, warm-start and progress-aware
  optimization concepts, without copying its dynamic model or solver stack.

This increment remains MPCC-lite.  It adds course-aware path topology to the
existing Frenet DP; longitudinal and steering inputs are not yet one nonlinear
optimization problem.

## Course-aware DP metadata

Each execution-corridor stage carries:

- reference curvature;
- whether the target footprint still constrains that stage.

Legacy samples leave both fields unset and retain the original DP cost.

## Tactical references

The DP compares the following reference families while using the same hard
corridor cells:

1. `StraightDashi`: preserve current lateral position with minimum motion.
2. `InsideDive`: move toward curve inside before the apex and open toward the
   outside after it.
3. `SweepDive`: enter from the outside, move toward the inside around the apex,
   then open to the outside after the target-active interval.
4. `OuterSweep`: retain the outside through the curve.

Contiguous same-sign significant-curvature samples define one curve segment.
The maximum absolute curvature sample is its apex.  The references use a
configurable fraction of the observed lateral extent. A reference may lie
outside one tactical branch so that the resulting mismatch is costed, but every
selected DP cell remains inside the supplied hard corridor. The normal DP slope
limit and downstream footprint validation still apply.

## Selection and continuity

For each side/homotopy, the strategy with the lowest complete DP cost wins.
The cost retains current-position anchoring, lateral-motion, previous-path and
corridor-width terms, then adds:

- distance to the tactical reference;
- a curvature/progress penalty for remaining on a slow inside radius.

The previous path remains a warm-start/hysteresis term.  A rolling refresh is
promoted only by the existing atomic target, horizon and hard-fault checks, so a
new tactic cannot erase the last feasible execution path merely by winning the
topology score.

## Scope after this increment

Still deferred:

- progress state `theta` as a controller state;
- joint acceleration/steering-rate nonlinear optimization;
- asynchronous solver workers;
- full Return integration into the same optimizer.
