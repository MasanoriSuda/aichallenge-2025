# Requirements

## Objective

Determine whether the existing outcome-qualified `speed_committed_teacher`
can resolve the packaged spatial policy's coherent side-hazard failure in the
same deterministic three-vehicle peer world.  This is a teacher admission
experiment, not a production fallback or parameter-tuning Slice.

## Frozen evidence

- Failed student run:
  `output/20260902-e2e-submission-freeze-peer-v2/d3`.
- The first sustained stall begins at `105.90 s` after bag start.
- During the preceding ten seconds, the packaged spatial model and diagnostic
  teacher have opposite mean residual signs on 23 side-hazard samples.
- Production raw/spatial identities, acceleration and peer scenario remain
  unchanged.

## Constraints

- Only d3 changes from `fixed_lidar_brake` to the already-implemented
  `speed_committed_teacher`; d1/d2 remain the frozen MPC peers.
- Teacher mode uses its certified no-speed-governor contract
  (`maximum_forward_speed_mps=0.0`).
- Do not change teacher thresholds, commitment state, acceleration, steering
  bounds, checkpoints, scenario, start state or Gate thresholds.
- Do not infer success from offline replay.  The teacher must execute the
  closed loop and pass Finish, penalty and stall Gates.
- Failed student commands are never used as teacher labels.
- Do not extract or train from this run unless a run-level outcome certificate
  verifies the exact bag/result/runtime identities.

## Definition of Done

- Focused controller and launch contracts remain green.
- The exact teacher mode and frozen model identities are visible in startup
  provenance.
- d3 finishes all three laps with zero penalties and zero post-start stall.
- If admitted, create a run-level certificate and an immutable teacher source;
  otherwise record rejection and stop before dataset/model changes.
