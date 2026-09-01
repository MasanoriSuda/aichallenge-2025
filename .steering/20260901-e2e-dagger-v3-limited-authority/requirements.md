# E2E DAgger v3 limited-authority requirements

## Objective

Run one deterministic closed-loop A/B with the shadow-admitted v3 spatial
candidate applying bounded steering correction.

## Frozen experiment

- base policy: production candidate3;
- control mode: `fixed_lidar_brake`;
- spatial candidate: DAgger v3 SHA
  `3b30f567d9a6bdf5384611ff8dfd759d79c8ed683c34e326e7d940afb2e67a5f`;
- authority bound: existing `0.12 rad`;
- no config, checkpoint, safety-distance or launch-default changes;
- production defaults remain spatial authority OFF.

## Acceptance

- Finish 3/3 laps;
- penalty count zero;
- stall durations zero;
- shadow/runtime errors zero;
- authority is applied and remains within `0.12 rad`;
- no material regression from the frozen single-vehicle baseline.

This run is evidence for or against v3 closed-loop authority.  A failure is not
authorization to increase the bound or add a runtime special case.
