# Exact ego/clock viability transition

Continue authorised completion after b19d6d7c. Production remains fixed while
investigating exact same376Accepted934and terminal-rejected935 from
20260909-peer-viability-dev2-r1 D1. Both real inputs reproduce without solving.
Peer-only counterfactual preserves both outcomes, falsifying peer update alone.
15ms change includes measured pose displacement~34.7mm, yaw -0.00856rad,
v1.267556->1.298703, current physicaldelta0.01462->0.02515, commanded0.09537->0.12036,
response-0.01383->+0.01334, control physicalprogress+0.033441m. These are measured
changes, not yet demonstrated producer defects. Compare controller input clocks
with same-run odometry/GNSS/IMU/steering/serialized commands. Do not tune noise,
thresholds, margins, delays, rates, gains or solver parameters to hide the loss.

935independentStop is accepted and constructs/adopts388. At937an ordinary376
input is Accepted; last publication ledger at938is current-world-bundle376/937.
938also inspects executedStop388, which lacks solver_source_snapshot and hence
cannot yet be directly replayed with the existing source-only decoder. Do not
claim that last publication376was exact-executed or replace388witha new solve.
Inspect the derived artifact/physical-proof provenance and current suffix
before choosing the minimum additional observation or repair.

Bounded offline sensitivities may replace named ego input groups while preserving
source artifact and all limits. Such hybrid inputs are counterfactual, not actual
measurements, feasible state transitions or promotable authority. Retain each
changed field and outcome. Only a failing causal producer invariant can select a
production repair. If formulation changes are proposed, run the architecture
comparison gate on the corresponding sealed world first.

Acceptance: failing native/replay before repair; appropriate package/build,
actual input replay and dynamic runtime; preserve previous rejected trials,
seal/register/spec/status/localcommit. No build/production edits during runtime.
Full moving Stop/restart/intents/dev3/dev4/gates/submission acceptance stays open.

## Installed EKF producer defect and selected repair

The actual installed library passes a fixed-clock native control but fails both
5ms mid-callback clock-advance cases: from state epoch9.98, dt0.02 predictsx0.02
at10.000, while stored/pose/twist epoch becomes10.005. The probe invokes the real
timerCallback with empty measurement queues and known constant velocity; only
Clock::now reads in this diagnostic process are controlled. It is not a runtime
shim. Six independent clock reads permit inconsistent calculation/publication.
The installed header exactly matches official upstream1e90380a3570d02e16d40287e0adf51a27cf34d0;
the corresponding source has the same separately sampled update/measurement/
pose/twist/publication clocks verified by installed symbols/disassembly.

Ego sensitivities on935: previous speeds alone accepts terminal+0.003454998m;
previous physical poses/progress accepts+0.031201191m. Previous physicalsteering/
response, predecessor command or executionclock alone do not rescue it. These
hybrid inputs are not real trajectories. The native EKF defect is established;
its full contribution to934/935and final938remains to be measured after repair.

Use a participant-owned aichallenge_ekf_localizer package based on the matched
historical implementation, with upstream attribution/manifest and existing
mathematical tests. Switch the participant launch/dependency to that package,
retiring the underlay EKF launch in the same change. Do not mutate the base image
or introduce a runtime interposition shim. Keep node/executable names, ROS topics,
service, QoS, parameters, filter math/noise/smoothing and submission contract.

Capture one positive-advance callback epoch; use it for dt, stored prediction
epoch, measurement delay and all state publications. TF carries the stored pose
stamp rather than restamping that older pose with a new clock. No added time
offset, timer rate, noise tuning, stale-data grace or control authority. Existing
clock-regression/reset behaviour is not newly claimed as validated by the
positive-advance regression. Test fixedclock and both mid-callback clock changes
against actual repaired node, plus upstream math tests, build and fresh single/
dev2 acceptance. Rollbackb19d6d7c. Full closure still needs derived388provenance,
controller observation-age consistency and broader runtime/submission gates.
