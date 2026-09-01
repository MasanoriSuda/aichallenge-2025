# Evidence

## Verification before runtime

- `make autoware-build`: 25 packages passed.
- ROS package/launch/system contracts: 80 tests passed.
- TinyLidarNet ML and analysis suite: 217 tests passed.
- Base checkpoint SHA, spatial checkpoint, authority bound, acceleration and
  4.6 m/s speed cap were unchanged.

## Four-vehicle A/B

- Frozen baseline: `output/20260902-e2e-final-packaged`
- Candidate: `output/20260902-e2e-final-speed-aware-safety`
- Only control-mode change:
  `fixed_lidar_brake` -> `speed_aware_lidar_brake`
- Integrated report:
  `output/20260902-e2e-final-speed-aware-safety/e2e-competition-analysis.json`

| world | baseline | candidate | classification |
|---|---|---|---|
| d1 | 0 laps, wall at 21.21 s, 180.21 s low speed | 4 laps, wall at 34.29 s for 7.13 s, 10.90 s low speed | partial recovery |
| d2 | 0 laps, wall at 21.11 s for 398.69 s | 0 laps, wall at 18.54 s for 401.28 s | regression / unchanged root failure |
| d3 | 4 laps, penalty 0, clean laps 84.99--85.45 s | 4 laps, penalty 0, clean laps 85.78--86.27 s | no stall, marginally slower |
| d4 | 4 laps, penalty 0, clean laps 84.68--85.89 s | 4 laps, penalty 0, clean laps 84.95--85.57 s | equivalent |

d2 remained at about `0.005 m/s` while frontal clearance was about 8.7 m and
the safety owner reported `clear`.  It was therefore not held by the new
longitudinal policy: positive acceleration could not move the physically
wall-bound vehicle.  This separates the lateral interaction failure from the
former `slow-clearance` zero-acceleration equilibrium.

## Decision

The integrated Gate is `fail`: no vehicle finished six laps, d1/d2 had wall
penalties and d2 remained stalled.  Do not change the production default.
Retain the explicit default-off mode and A/B target because it removes the
structural zero-acceleration fixed point and produced a real d1 recovery, but
return the main investigation to interaction-aware lateral data/authority.
