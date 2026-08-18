# Requirements

## Purpose

The asynchronous tactical worker currently generates complete left/right
Frenet-DP Missions, but only the Mission selected by the discrete tactical
layer reaches the five-state/three-input extended MPCC.  Evaluate both complete
Missions with the same extended MPCC formulation before atomically selecting a
side.

## Requirements

- Run left and right branch evaluation concurrently only in the existing
  latest-only worker.
- Use the production extended MPCC state, input, stage corridor, wall and
  velocity constraints for both branches.
- Publish both branch results in one immutable tactical result.
- Select only a finite, solved branch with the configured minimum physical
  bound reserve.
- Preserve the current side after the tactical no-return point.
- Require a configurable objective advantage before switching away from an
  already committed feasible side.
- If both branch solves fail or are unavailable, retain the existing tactical
  result and the current last-feasible execution path.
- Keep the 40 Hz control callback non-blocking.
- Do not change ROS topic, service, launch or evaluation contracts.

## Non-goals

- Separate-process worker isolation.
- Recovery/Reverse changes.
- Dynamic bicycle or tire-force model.
- Replacing the existing low-level 40 Hz extended MPCC solve.
