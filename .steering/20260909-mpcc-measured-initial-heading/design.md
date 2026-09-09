# Measured initial vehicle heading

Continue M1–M6 from094941b5; participant source1c4f377e. Full completion stays open.
Force-step body-model selection found an earlier current-state defect: dev2 pre-E
odometry yaw differs from same-source physical yaw by up to0.3854038rad (D1).
The node initializes using the nearest reference-path heading, unrelated to the
actual starting body orientation. GNSS also derives its initial lever-arm pose
from an unobserved previous orientation until displacement exceeds0.2m.

The local raw IMU supplies absolute orientation. Same-source quaternion comparison
against Rigidbody converted with the local ROS2Utility mapping identifies a fixed
body-to-IMU yaw of+pi/2 (1124observations, max rotation residual1.11e-7rad).
The simulation sensor TF currently declares-pi/2, a180degree mismatch. Verify source
frames, static TF and exact IMU/GNSS epochs in bags before implementation.

Repair scope: simulation-specific IMU extrinsics; same-source GNSS position and
transformed IMU attitude before lever-arm conversion; measurement-based initial
pose and the same /set_initial_pose service path. Use standard ROS sensor topics,
never private Unity state for control. Delete untransformed raw-IMU fallback.
Keep vehicle configuration and its explicitly selected initialization mode separate.
No covariance/gain/limit/solver/timeout tuning. Preserve service and topic contracts.

Before production edits, native/frame and recorded-source counterexamples must fail
old behavior. Validate transformed frame, quaternion validity, unavailable/mismatched
source epochs, no input mutation, initial heading independent of path geometry and
same-source pose/lever arm. Then build affected packages, tests, actual-node replay,
single/dev2 runtime and final authority. Run no edits/build alongside simulation.
Architecture comparison on928already rejects all9normal/4Stoparms with old model;
this is Unknown, not physical infeasibility. No candidate architecture changes here.
Rollback094941b5; force-step model and full M2–M6 remain subsequent required work.
