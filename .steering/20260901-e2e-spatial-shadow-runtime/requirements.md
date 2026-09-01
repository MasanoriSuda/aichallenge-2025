# Spatial adapter shadow-runtime requirements

## Objective

Execute the offline-gated spatial-plus-speed candidate beside candidate3 during
a real three-lap run without granting it steering authority.

## Frozen authority

- published steering and acceleration remain candidate3 +
  `fixed_lidar_brake`;
- spatial output is diagnostic only;
- an absent shadow checkpoint leaves the production inference path unchanged;
- shadow inference failure must never replace a valid production command.

## Contracts

1. NumPy runtime output matches the PyTorch reference for the same strict
   checkpoint and input.
2. The artifact embeds an exact copy of candidate3; a mismatch is rejected at
   startup.
3. Speed-enabled shadow inference requires a finite, non-negative and fresh
   `/localization/kinematic_state` sample.  Missing speed skips shadow inference
   rather than substituting zero.
4. Every shadow interval logs coverage, correction distribution, direction
   probabilities, errors and total callback inference time.
5. Optional launch plumbing is empty by default and preserves all existing ROS
   topic and submission contracts.

## Acceptance

- three laps and zero penalties under unchanged production authority;
- no production command divergence relative to the shadow-disabled code path;
- at least 99% shadow coverage after startup;
- finite correction on every admitted shadow sample;
- 20 Hz scan handling with no watchdog activation and sufficient inference
  capacity;
- generated evidence records candidate SHA and exact runtime configuration.
