# Evidence

# Evidence

## Frozen run

- run: `output/20260902-e2e-final-packaged`
- baseline: `2d0680eb`
- runtime: `fixed_lidar_brake`, acceleration `0.8 m/s2`, forward-speed cap
  `4.6 m/s`
- model: packaged TinyLidarNet plus packaged spatial authority
- world: four Autoware vehicles, six required laps, 420 s timeout

All four launch logs contain the same packaged checkpoint path and the same
runtime values.  LiDAR remained at about 10 Hz and no stale input or inference
failure was reported.  AWSIM wrote the authoritative result JSON before any
container was stopped.

## Competition result

| Domain | Finish | Penalty | Mean / max speed | Motion result |
|---|---:|---:|---:|---|
| d1 | 0/6 | wall 1, 6.875 s | 0.101 / 3.896 m/s | fail; 180.209 s low speed |
| d2 | 0/6 | wall 1, 398.689 s | 0.100 / 3.782 m/s | fail; 180.217 s low speed |
| d3 | 4/6 | 0 | 3.731 / 4.464 m/s | motion pass, race fail |
| d4 | 4/6 | 0 | 3.724 / 4.473 m/s | motion pass, race fail |

d3 completed four laps in `91.649 / 85.448 / 85.198 / 84.993 s`; d4 completed
four laps in `87.187 / 84.793 / 84.683 / 85.887 s`.  Neither can complete six
laps inside 420 s at this pace.  The competition analyzer rejected all four
domains after the runtime identity was matched to the packaged checkpoint.

## Earliest causal failure

d1 and d2 both travelled about 48 m before the first wall events at race time
`21.211 s` and `21.111 s`.  They then remained in `stop-clearance` braking with
fresh LiDAR and finite ML steering.  The frozen bag contexts show:

- d1: front `0.64--0.71 m`, closest/right `0.62--0.65 m`, command acceleration
  `-1.0 m/s2`;
- d2: front `0.38--0.39 m`, closest/left `0.262 m`, command acceleration
  `-1.0 m/s2` and steering `-0.64 rad`;
- stopped poses are about `2.73 m` apart.

This is not a checkpoint-loading, sensor-stale, inference, launch or speed-cap
failure.  The controller reaches a physically trapped state before its
distance-only longitudinal safety can stop the approach, and steering alone
cannot move a stationary Ackermann vehicle out of that state.  The first
bounded correction is therefore speed-aware pre-contact braking; lateral
policy/data improvement remains a separate follow-up and may not be hidden by
loosening the stop threshold.

## Independent pace failure

d3 and d4 demonstrate a second failure that is independent of contact.  They
have no penalties or stalls, but their best laps remain about `84.7--85.0 s`.
The current `4.6 m/s` cap was qualified only inside the existing lateral
policy's evidence envelope; raising it without higher-speed teacher data would
repeat the previously rejected unbounded-pace experiment.  Pace remains frozen
until the safety Slice and new higher-speed data Gate are complete.
