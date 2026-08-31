# Dynamic evidence

## Rejected front-only teacher

- Run: `output/20260901-040203`
- Result: rejected; positive-acceleration stall for 40.16 s after 202.67 m.
- The stall began at `(89648.32, 43172.07)` with the left side at 1.32 m and the
  right side above 5 m. The command still requested `+0.6 m/s^2` and `+0.44 rad`.
- The original fixed baseline `output/20260901-034202` stalled at the same physical
  location after 203.77 m. The gap teacher therefore did not introduce a new failure;
  it failed to retain authority when the obstacle moved from the front sector to the
  side sector.

## Admitted side-aware teacher

- Run: `output/20260901-041154`, seed 2026.
- AWSIM state: `Finish` after three laps.
- Distance: 1015.94 m; maximum speed: 4.57 m/s.
- Longest low-speed interval after motion: 0.0 s.
- Longest positive-acceleration stall: 0.0 s.
- No crash/collision/recovery string was emitted by AWSIM or Autoware.

The structural change was to retain LiDAR teacher authority for lateral wall risk and
steer away from the constrained side. Production remains `control_mode=fixed`; this
run is only an admitted teacher candidate.

## Independent teacher validation

- Run: `output/20260901-041940`, seed 2027.
- AWSIM state: `Finish` after three laps.
- Distance: 1018.80 m; maximum speed: 4.56 m/s.
- Longest low-speed and positive-acceleration stall: 0.0 s.
- The two admitted runs were split by run identity: seed 2026 is train (6011
  samples), seed 2027 is validation (6130 samples), with zero synchronization
  rejection and `label_source=lidar_gap_teacher`.

## First imitation candidate

- Checkpoint: `checkpoints/20260901_042829/candidate.npy` (not promoted).
- Independent validation overall RMSE improved by 17.96%, while overall MAE
  regressed by 28.06%.
- On the 628 corrective samples where teacher and production differ by at least
  0.02 rad, MAE improved from 0.0965 to 0.0597 rad and RMSE improved from
  0.1593 to 0.1211 rad.
- Single-vehicle run `output/20260901-043146` reached `Finish`: 1006.34 m,
  mean 3.39 m/s and no positive-acceleration stall.
- NPC run `output/20260901-044333` failed admission after 888.27 m. It remained
  below 0.15 m/s under +0.6 m/s^2 for 59.73 s. A preceding infrastructure-only
  run `output/20260901-043847` was invalid because AWSIM stopped publishing
  clock and sensor topics in `Spawned` state.

At the failed pose near `(89660.58, 43162.19)`, both successful teachers passed
with approximately +0.20 to +0.28 rad steering and metre-scale clearance. The
student reached the area with a different NPC geometry, drove steering toward
-0.52 rad as the left return approached, and eventually received near-zero
LiDAR returns on both sides. This is a closed-loop distribution-shift failure,
not evidence for changing the teacher thresholds. The next slice should relabel
pre-contact student states with the same teacher (DAgger) and must exclude the
physically embedded suffix.
