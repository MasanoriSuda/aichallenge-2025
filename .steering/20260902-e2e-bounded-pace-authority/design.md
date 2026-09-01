# Design

## Longitudinal authority

Add a deterministic proportional governor before the existing LiDAR
longitudinal safety layer:

```text
requested_acceleration
  -> speed governor: min(requested, max(0, v_limit - v))
  -> LiDAR slow/stop safety
  -> final acceleration command
```

The proportional coefficient is `1.0 /s`, so the difference between the
configured limit and current wheel speed is numerically an acceleration cap.
The governor never raises acceleration and never clips a negative brake
request.  It is disabled by `maximum_forward_speed_mps <= 0`.

## Data flow

```text
TINY_LIDAR_MAXIMUM_FORWARD_SPEED_MPS
  -> system launch
  -> submit launch
  -> reference launch
  -> controller launch
  -> TinyLidarNetCore
```

Wheel speed is already synchronized into the core for the qualified spatial
adapter.  Enabling the governor makes fresh speed mandatory even if a future
lateral model no longer consumes speed.

## Evaluation

Use explicit `acceleration=0.8` and `maximum_forward_speed=4.6` on actual seed
2035 after verifying the AWSIM process argument.  Only after Finish, zero
penalty and zero stall may the same candidate run on an independent seed.
Production defaults do not change before both Gates pass.
