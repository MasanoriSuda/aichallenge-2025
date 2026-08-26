# Audit

## Observed phenomenon

Run `output/20260827-020001` completed six laps, but laps 3--7 repeatedly lost
most physical speed near path segments 29--30. The first event changed EKF
speed from 8.738 to 5.013 m/s in 19 ms; the raw vehicle report was already
0.134 m/s while the preceding command still requested `+1.366 m/s2`.

`aichallenge/d1-result-details.json` identifies seven `wall` penalties totaling
about 82.4 seconds, with `crash=0` and `over=0`. Decompilation of the shipped
AWSIM `Assembly-CSharp.dll` proves that any active penalty clamps horizontal
rigid-body speed to exactly 1.388889 m/s. Wall contact holds its trigger for at
least five seconds. The 5 km/h plateau is therefore an AWSIM Wall penalty, not
an MPCC braking command.

## Causal chain

1. The GNSS pose arrives with a current producer timestamp.
2. The launch nevertheless tells EKF it is another 0.3 s old.
3. EKF advances the curved-path pose by approximately `v * 0.3`.
4. The controller separately predicts 0.13 s for measured actuator response.
5. MPCC and the map wall proof are internally consistent at that advanced map
   pose, while AWSIM checks the actual rigid body.
6. AWSIM detects wall contact, applies Wall penalty, and clamps speed to 5 km/h.
7. Emergency, reverse, and long laps occur downstream of that physical clamp.

The 0.3 s value came from
`.steering/20260720-mpc-localization-delay-compensation`, whose requirements
explicitly state it was a provisional 2025 value without ground-truth timing
evidence. It is past technical debt, not a measured interface contract.

The independent content-latency check uses GNSS position finite differences,
raw vehicle speed, GNSS yaw finite differences, and raw IMU yaw rate. Across
6014 moving samples, speed error is smallest at zero shift (median 0.028 m/s)
and increases monotonically to 0.100 m/s at 0.30 s. Yaw-rate error is smallest
at 0.05--0.10 s (median 0.093/0.084 rad/s) and reaches 0.391 rad/s at 0.30 s.
The bag therefore directly rejects a 0.30 s measurement-content delay. The
remaining yaw response agrees with the separate 0.13 s controller/actuator
model and is not evidence for advancing the EKF pose.

## Rejected or weakened alternatives

- Controller braking: rejected by the positive causal acceleration command.
- Acceleration anomaly: rejected by AWSIM's 3.0 threshold and `over=0`.
- Vehicle collision: rejected by the single-vehicle run and `crash=0`.
- Fixed impassable wall point: weakened because an upper-ranked bag passes
  within 3 cm of the same reported map pose at 9.34 m/s.
- Lateral acceleration alone: weakened by comparable upper/clean passages.
- Rollover: rejected by near-zero IMU roll/pitch before impact.
- AWSIM wall-recovery rotation: not the initiator; dev uses
  `--wall-recovery off`.

## Implemented repair

- Changed the simulation EKF `pose_additional_delay` default from `0.3 s` to
  `0.0 s`; the explicit launch override remains available for a future measured
  sensor delay.
- Kept the independently identified `0.13 s` controller/actuator prediction.
- Added a launch-contract regression test which failed first on the old
  `0.3 s` default and now enforces zero defaults plus explicit EKF forwarding.
- Updated the integration specification to keep sensor measurement timing and
  actuator response timing as separate responsibilities.

No wall margin, speed, steering weight, timeout, Recovery, or fallback value
was changed in this slice.

## Static verification

- `test_localization_timing_contract.py`: `2 passed`.
- `make autoware-build`: `25 packages` completed successfully.
- Focused `colcon test` for `aichallenge_submit_launch` and
  `multi_purpose_mpc_ros`: `1915 tests`, zero errors/failures/skips.
- Both analysis helpers pass `python3 -m py_compile`.

## Dynamic proof

Run `output/20260827-030029` was executed as a single-vehicle six-lap
Track/Cruise Gate after the repair.

| Metric | Before: `20260827-020001` | After: `20260827-030029` |
|---|---:|---:|
| laps | 6 | 6 |
| lap times [s] | 43.235, 40.619, 75.263, 61.994, 88.971, 60.679 | 44.780, 41.640, 41.735, 41.240, 40.955, 41.495 |
| maximum lap [s] | 88.971 | 44.780 |
| AWSIM Wall penalties | 7 | 0 |
| crash / over penalties | 0 / 0 | 0 / 0 |
| control callback overruns | none observed | 0 |

The repeated event-curve comparison provides the stronger local proof. Before
the repair, passages 3--7 contain abrupt single-sample losses of
`3.27--6.88 m/s`, negative/recovery speed, and EKF-to-header-aligned GNSS
separation around `2.0--2.4 m`. After the repair, all seven recorded passages
complete in `6.14--8.49 s`, remain above `7.86 m/s` after the first launch
passage, and have no abrupt loss above `0.17 m/s`. Header-aligned GNSS
separation at the event region is `0.03--0.49 m`.

The independent content-latency check on the repaired bag again rejects a
`0.3 s` measurement delay: speed median error is `0.031 m/s` at zero lag and
`0.089 m/s` at `0.30 s`; yaw-rate median error is best at `0.05--0.10 s`
(`0.087/0.083 rad/s`) and worsens to `0.450 rad/s` at `0.30 s`.

## Conclusion

The failure-first test, producer-timing evidence, local replay comparison, and
clean six-lap dynamic Gate agree. The unmeasured EKF delay was the root
producer of the map/physical-pose mismatch that allowed a self-consistent MPCC
wall proof to drive the rigid body into AWSIM's wall collider. Removing that
legacy timing patch fixes the invariant at its owner instead of masking the
downstream penalty with controller tuning.

Generated result JSON and rosbag artifacts are evidence only and are not part
of the commit.
