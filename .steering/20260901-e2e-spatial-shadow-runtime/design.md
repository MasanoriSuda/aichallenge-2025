# Design

## Runtime structure

```text
LiDAR -> candidate3 -> published steering
   |          |
   |          +-> longitudinal LiDAR safety -> published acceleration
   |
   + speed -> spatial adapter -> shadow metrics only
```

`SpatialSteeringAdapterNp` mirrors the offline signed-mixture head: embedded
base conv5 map, immutable projection, fixed train statistics, normalized speed,
two ReLU layers, direction softmax and signed magnitude mixture.  Its complete
29-tensor checkpoint is loaded strictly.

The core first computes the production output.  Shadow computation is isolated
in a guarded block and stores diagnostics but never mutates `steer` or `accel`.
At startup, every embedded `base_*` tensor is compared exactly with the loaded
production checkpoint so the correction cannot silently target another base.

## Speed synchronization

The ROS node subscribes to `/localization/kinematic_state`.  At each LiDAR
callback it passes speed only when its receive age is within a configured
100 ms bound.  This is a live shadow diagnostic, not a training sample; bag
timestamps remain the offline evidence authority.

## Failure isolation

- invalid checkpoint/config: fail startup;
- missing/stale speed: skip shadow, publish production;
- runtime shadow exception/nonfinite output: count/log error, publish
  production;
- LiDAR/base failure: retain the existing stop behavior.
