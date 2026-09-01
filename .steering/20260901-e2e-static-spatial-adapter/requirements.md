# Requirements

## Objective

Test one frozen-base, LiDAR-only static spatial correction candidate after the
action-separability Gate rejected compact and temporal alternatives.

## Constraints

- production candidate3 and `fixed_lidar_brake` remain unchanged
- no odometry, V2X, map, trajectory or teacher command enters inference
- candidate3 conv layers are immutable and bit-identical in the artifact
- full `conv5` geometry is retained; no 10-dimensional policy bottleneck
- train/validation sequence identity and peer-d3 validation assignment remain
- no runtime shadow or closed-loop run until every offline Gate passes

## Definition of Done

- zero-initial adapter is exactly candidate3 for every scan
- continuous material correction improves at least 30%
- material direction accuracy is at least 80%
- validation anchor and independent normal MAE are at most 0.01 rad
- peer-d3 direction and anchor Gates pass with support disclosed
- failed candidate is retained only as diagnostic evidence
