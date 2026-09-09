# Measured initial heading: repair and acceptance

Baseline094941b5; MPCC source remains1c4f377e. Full MPCC completion is not achieved.

The first broken invariant is the source body pose. Simulation declares body→IMU
yaw−pi/2 but1124same-source Rigidbody/IMU samples require+pi/2, with maximum
quaternion residual1.108938698e-7rad. Old transformed attitude is wrong by pi.
Initialization also substitutes path tangent for actual heading, while GNSS uses
an unobserved previous heading for the antenna lever arm until enough movement.
The original counterexample report is retained, exit1. Bags did not record
tf_static or initial_pose3d; generated complete URDF chains subsequently confirm
the simulation+pi/2 and unchanged vehicle−pi/2 transforms.

Repair: simulation-only extrinsic; exact-source GNSS/IMU pair; normalized attitude
transformed to GNSS frame before antenna→body lever arm; measurement-based startup
and /set_initial_pose using one function and original source stamp. Missing/invalid
attitude or TF fails closed. Pairing stores only latest/pending samples, accepts
either arrival order and rejects stale/future substitution. The raw GNSS fix-status
topic remains independent of IMU availability. The untransformed raw-IMU fallback
and initializer IMU subscription/cache are deleted. Vehicle explicitly keeps its
existing raceline initialization. Topic/service names and types stay compatible.
No gain, covariance, solver, safety margin, delay or authority changes.

Validation commands and results (logs sealed in evidence.json):

- `make autoware-build`: final r4,26packages successful. No source edits during build.
- Container `colcon test --packages-select racing_kart_gnss_poser` and
  `colcon test-result --verbose`: final39records,0errors/0failures,7cppcheck-wrapper
  skips. Six new IMU/source/TF/fix-status native cases; original heading/covariance
  cases retained. Initializer11cases pass; MPCC60CTestgroups/2381records pass.
- Direct cppcheck r2 and final r4 find only an unchanged heading_reference.cpp:42 style
  suggestion, independently reproduced from baseline. It exits1, not clean0.
  The initial new test shadowing and copyright failure were fixed and rechecked.
- Both simulation and vehicle `ros2 launch --show-args ...` parse. Initial r1
  unsupported conditional param failure is preserved; conditional let repairs it.
- Actual GNSS and initializer node replay r4:138/138D1 and143/143D2 source pairs,
  attitude error below9.43e-16rad. All output poses and both initializations exactly
  equal successful r2. Early service refuses; automatic and explicit service retain
  first/last sensor epochs. Replay accelerates wall time, not executor parity.
  r3 fixture TF startup race is preserved; test fixture now waits for static TF
  delivery before sensor injection. Production missing-TF behavior stays fail-closed.

Uninstrumented single-r2:6laps251.15843200683594s, penalty0, no moving override.
Full recording300windows/12169cycles, callback maximum18.406ms,0overruns.
Dev2-r1: first moving D1Emergency933/source10.309999769,1.8082220948765995m/s in
ready phase. Both initial headings2.127rad. This coupled run rejects; no finish.
Both runs use buildr3, before the final independent fix-status notification move;
r4 identical pose replay covers that move. Final same-HEAD campaign remains open.

Same-source calibrated raw-IMU heading comparison during moving pre-E windows:
D1mean absolute error0.28548396735->0.00208410582rad; D2
0.14361290444->0.00535046991rad. This compares separate closed-loop windows against
IMU, not independent Rigidbody truth for the new runs. Odom speed still differs
from raw COM forward speed by mean0.04988/0.04857m/s.

The new933world is fp18084597137145989003, actual/inspectedartifact382. It must
not be mixed with older933/artifact371 or force-run928. Strict source-free
zero-solve revalidation reproduces original certificate and previous/current
outcomes. At932 terminal peer clearance+0.00239983282409m; at933
−0.0000303259375765m. Independent Stop rejects−0.00178549315808m. Wall and steering
join pass. At933 control-origin speed1.91152601996m/s versus artifact2.21718518754,
pose join error0.11216139m. Peer-only substitutions preserve both outcomes, so
peer observation change alone does not explain the transition. All9normal and
4Stop architecture arms reject/Unknown. No physical-infeasibility claim.

Review: no raw IMU attitude bypass, no silent identity transform in IMU mode,
source-stamp and service responsibilities preserved, simulation/vehicle profiles
parse, generated vehicle TF unchanged. Runtime artifacts restored to their original
user hashes; no outputs/bags/binaries committed. Rollback094941b5. Remaining work:
shared body/application/rest model, complete proof/schema replacement, all intents,
multicar/gates and same-artifact submission/eval. Continue that work autonomously.

Registry validation passes130snapshots/270experiments. Python/JSON/XML/YAML, new
result/status links, bash syntax and git diff whitespace checks pass. Host pre-commit
is unavailable; equivalent relevant checks run directly. No new Skills changed.
