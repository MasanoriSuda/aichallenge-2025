# Evidence

## Controlled A/B

- command: `make e2e-final-contact-teacher`, then
  `make awsim-request-start` after all four domains reported `Grounded`
- run: `output/20260901-090729`
- d1--d3: production `tiny_lidar_net + fixed_lidar_brake`
- d4: existing teacher-only `tiny_lidar_net + gap_teacher`
- world, start positions, longitudinal policy and production checkpoint were
  unchanged from `output/20260901-085903`

| domain | lateral owner | distance | duration | longest low speed | positive-accel stall | result |
|---|---|---:|---:|---:|---:|---|
| d1 | production student | 660.85 m | 272.10 s | 0.00 s | 0.00 s | pass |
| d2 | production student | 51.55 m | 280.11 s | 156.83 s | 116.42 s | fail |
| d3 | production student | 100.23 m | 289.51 s | 84.18 s | 84.18 s | fail |
| d4 | gap teacher | 108.31 m | 300.28 s | 84.42 s | 84.42 s | fail; later escaped |

The d4 launch log proves `tiny_lidar_control_mode: gap_teacher`.  The teacher
also took lateral authority repeatedly: the status changed from `front-clear`
to `side-clearance` as the right-side 10th-percentile distance fell below
1.8 m, and commanded an increasing positive escape steer.  It therefore did
not fail because the diagnostic target accidentally ran the production mode.

## Root-cause boundary

The existing teacher computes side risk from only the extreme side sectors
(`abs(angle) >= 1.3 rad`) and the current 10th-percentile range.  It has no
relative-motion estimate, time-to-side-contact, temporal occupancy tube or
vehicle-to-vehicle association.  Once the front sector is clear it blends the
base steering with a reactive side escape only after the current side distance
crosses 1.8 m.

In the final-world run, that reactive correction remained active for many
seconds and eventually helped d4 escape, but it could not prevent an 84.42 s
physical trap.  Increasing the same threshold would not establish that the
steering direction is correct; it could turn track walls and harmless nearby
returns into false peer threats.  This experiment therefore rejects both of
the following shortcuts:

1. extracting this failed d4 sequence as corrective teacher data;
2. promoting `gap_teacher` or its side threshold into production.

## Decision

The corrective-data branch is **not admitted**.  The next teacher must be a
separate pre-contact design that reasons about temporal LiDAR motion and
preserves a collision-free escape over a short horizon.  It must first run in
shadow/diagnostic mode and pass the same final-world stall gate before any
labels are extracted.  Production checkpoint, longitudinal safety thresholds
and lateral authority remain frozen.

## Checks

- `make -n e2e-final-contact-teacher`: correct d1--d3 production and d4 teacher
  launch contract
- launch logs: all four domains used the expected controller mode
- all four bags finalized before analysis
- analyzer: d4 failed the run-level positive-acceleration stall gate
- `git diff --check`: clean
