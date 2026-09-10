# Candidate review before fresh runtime acceptance

Baseline `6b0f1880`; this review covers the shared nine-state migration slice.
It does not certify integrated race completion.

- Earliest structural mismatch: wire acceleration was used as net COM acceleration,
  while the observed wheel/drag/body dynamics differ. The fixed native replay gives
  2.074699099m/s after wire1 from2m/s for0.2s; the replaced affine law gives2.2m/s.
- One kernel now owns tire response, four-wheel planar body propagation and nominal
  settled rest. Prefix, tangent, nonlinear proof, execution cursor and Stop use it.
  Sensor source epochs, actual serialized publication history and model fingerprint
  travel with the nine-state request/artifact and async compatibility context.
- Deleted normal paths: velocity-affine tangent override, yaw-rate/speed steering
  inversion, separate longitudinal response observer, exponential-yaw prefix,
  wire-times-duration reachable-speed certificate. Historical diagnostic interfaces
  remain where required to read earlier evidence.
- Detection gap found by regression: a QP zero endpoint could coexist with physically
  moving replay. Exact terminal body rest is now a sealed physical proof obligation;
  existing bounded SQP correction handles its failure. Positive terminal wire is
  excluded by the Stop producer before solving because it exits the nominal rest mode.
- Frozen rest-mode comparison: direct-primal and rollout-state relinearization each
  succeed after one correction; four held-speed/braking stage alternatives succeed
  with the same hard nonlinear oracle. The direct failed candidate is not infeasible.
- State-count fixtures, model hashes and physically invalid tire/short-horizon Stop
  fixtures were migrated. No failed case was skipped. Limits, margins, physical
  tolerances, solver iteration settings and maximum three proof corrections remain.
- Public-input holdout error is empirical: single1s position MAE0.193661302m,
  maximum0.675902423m. Unknown contact/application timing remains unbounded.
  The old dev2 bag ends before restart and cannot establish restart acceptance.
- Remaining authorities: solved/certified normal MPCC with bounded same-context
  retained execution; Emergency and separate contact/gear/reverse Recovery.
- Remaining runtime concerns: actual public-input coverage/QoS, callback cost,
  prediction error and all intents. ROS clock reset recovery of sensor/history
  buffers needs a causal exercise during restart/async coverage; it is not a pass.
- Rollback: focused revert of this migration commit, preserving localization9bca3af6,
  protected user JSON, simulator binary and all recorded runs.

Build/package/pre-commit/native/public comparisons are bound by `evidence.json`.
Fresh single/dev2 and the remaining M3-M6 checks are intentionally still open.
