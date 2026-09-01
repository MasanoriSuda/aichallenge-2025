# Evidence

## Immutable production baseline

- Base TinyLidarNet SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- Default control mode: `fixed_lidar_brake`
- Default residual checkpoint path: empty (disabled)

## Diagnostic residual candidate

The candidate used for closed-loop diagnosis was
`821bf57d9f213a39f748fd9eb3717a9a3a15c873b24a1f614afee573e674c2e9`.
It was selected by material, anchor and independent-normal offline gates; it is
not a production artifact.

| Gate | Run | Result |
|---|---|---|
| single vehicle | `output/20260901-120424` | pass; 1015.22 m, 299.20 s, mean 3.40 m/s, stall 0 s |
| runtime NPC | `output/20260901-121209` | pass; 1020.53 m, 318.60 s, mean 3.20 m/s, stall 0 s |
| four peer | `output/20260901-121938` | reject; d1/d2 stall from about 118 s |

Four-peer per-domain result:

| Domain | Distance | Mean speed | Positive-acceleration stall |
|---|---:|---:|---:|
| d1 | 109.63 m | 0.33 m/s | 211.74 s |
| d2 | 104.16 m | 0.31 m/s | 219.02 s |
| d3 | 1049.37 m | 3.00 m/s | 0 s |
| d4 | 1102.46 m | 3.05 m/s | 0 s |

d1/d2 continued to publish +0.6 m/s2 while stopped.  Their final positions
were about 1.9--2.2 m apart, so this is a lateral interaction failure rather
than an intentional longitudinal stop.

## Root-cause evidence

- Existing candidate on new d2 pre-failure prefix: material improvement 9.4%.
- Existing candidate on final 10 s: material improvement 0.05%.
- Required final-10-s material correction:
  - d1 mean `+0.346 rad`
  - d2 mean `-0.850 rad`
- Nearest d1/d2 normalized LiDAR states had mean RMS distance 0.044 and teacher
  sign agreement 0%.
- Equal-sequence sampling and longer optimization did not learn these states
  without normal-anchor regression.
- Component diagnostics rejected gate collapse: material gate means were 0.98
  (d1) and 0.89 (d2), while the ungated correction itself remained near zero.

Classification: the current stateless single-frame residual does not robustly
separate the opposite homotopies near the interaction boundary.  Runtime
threshold, brake-distance and production base changes are not justified by
this evidence.

## Verification

- TinyLidarNet ML tests: `66 passed`
- TinyLidarNet runtime tests: `30 passed`
- submit launch tests: `5 passed`
- system interface tests: `7 passed`
- `make autoware-build`: `25 packages finished`, successful

Generated datasets, checkpoints, rosbag/output artifacts and result JSON files
remain outside the commit.
