# Evidence

## Deterministic single vehicle

- run: `output/20260901-151131`
- result: pass
- Finish: 3/3 laps
- laps: 99.030 / 88.241 / 88.956 s
- penalties: 0
- distance: 1007.75 m
- post-start low speed: 0.00 s
- positive-acceleration stall: 0.00 s

## Deterministic runtime NPCs

- command: `make e2e-npc-single`
- seed: 2026
- run: `output/20260901-152109`
- result: fail
- Finish: no, 2/3 laps at the 420 s timeout
- laps: 100.819 / 87.986 s
- penalties: one wall event, 137.39 s
- distance: 906.33 m
- post-start low speed: 117.05 s
- positive-acceleration stall: 117.05 s
- first persistent stall context:
  - sim-relative start: 288.25 s
  - command acceleration: +0.6 m/s2
  - command steering: -0.0169 rad
  - front / right-front / right-side: 1.05 / 0.47 / 0.36 m
  - pose: `(89665.91, 43164.77)`

The run reproduces a third-lap wall/contact embedding while inference and
positive acceleration continue.  It is not a sensor-stale event or an
intentional `fixed_lidar_brake` stop.  The prior motion-only NPC success is not
sufficient evidence of deterministic acceptance.

AWSIM's NPC summary gives all three vehicles `vehicle_number=1`.  The v3
per-domain detail remains the identity authority; the v2 summary is accepted as
a cross-check when one entry has the same Finish/lap state.  This contract was
added to the analyzer and covered by a regression test.

## Four production peers

- command: `make e2e-final`
- seed: 2026
- run: `output/20260901-153143`
- result: fail
- session timeout: 420 s

| domain | Finish | laps | best lap | penalties | distance | longest low speed |
|---|---:|---:|---:|---:|---:|---:|
| d1 | no | 0/6 | N/A | wall x1, 361.79 s | 49.53 m | 120.07 s |
| d2 | no | 4/6 | 91.065 s | 0 | 1680.06 m | 0.00 s |
| d3 | no | 3/6 | 89.975 s | wall x1, 77.02 s | 1355.14 m | 85.24 s |
| d4 | no | 4/6 | 89.650 s | 0 | 1654.69 m | 0.00 s |

d1 entered a persistent right-side wall/contact embedding.  At the start of
its 79.82 s positive-acceleration stall, the command remained +0.6 m/s2,
front LiDAR remained open at 8.02 m, and right-side clearance was 0.68 m.  At
bag end, front remained open at 8.56 m and right-side clearance was 0.61 m.
This is lateral policy failure, not front-brake activation or inference loss.

d3 incurred its wall event on lap 1 and remained below 1 m/s for 85.24 s.  d2
and d4 avoided measured stalls and penalties, but their approximately 90 s
clean laps were too slow to complete six laps before timeout.  The matrix
therefore shows two independent gaps: lateral contact robustness and clear-lap
pace.  It does not support a runtime threshold patch or checkpoint promotion.

## Frozen artifact verification

- training candidate:
  `aichallenge/ml_workspace/tiny_lidar_net/checkpoints/20260901_055824/candidate.npy`
- packaged source checkpoint:
  `aichallenge/workspace/src/aichallenge_submit/tiny_lidar_net_controller/ckpt/tinylidarnet_weights.npy`
- SHA-256 for both:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- runtime control mode: `fixed_lidar_brake`

## Bounded next slice

Before another training run, compare the pre-contact LiDAR/velocity states from
the NPC and peer failures with the frozen candidate's admitted train/validation
sequences and seed-disjoint teacher runs.  The decision must distinguish:

1. out-of-distribution states: collect teacher data at the missing states;
2. observation aliasing: the same single scan needs opposite action, requiring
   an allowed temporal/odometry representation rather than more copies;
3. covered but wrong states: inspect optimization and label provenance.

No production model or runtime behavior changes in that audit.

## Verification

```text
python3 -m py_compile analyzer and analyzer tests     pass
TinyLidarNet full tests in autoware-command           95 passed
git diff --check                                      pass
```
