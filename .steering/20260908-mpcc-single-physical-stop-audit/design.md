# Causal boundary

1. Source250.599994s vehicle-status7.796716m/s;250.634994s0.852230m/s.
   Receive1788798709.467563→1788798709.501314. This precedes the command
   reduction to2.279889m/s at receive1788798709.552788/source250.684994.
   Commands at250.604994/250.634994/250.654994 remain7.79/7.80/7.80m/s,
   acceleration+1.3296. Immediate command braking is falsified.
2. Filtered odometry lags the sudden sensor change; normal solutions remain
   current-world certified and adapt to near-zero measured velocity.
3. Stuck confirmed at1788798711.253979, evidence-free qualification, no
   reported wall contact. Recovery10713 publishes HoldStop;10714 then fails
   steering join after the separate supervisor interrupted the ledger.
4. Reverse motion stops after2.563m because rear information is incomplete;
   no normal resumption. Wall penalty is reported at race242.938812s, later
   than the first sudden slowdown (about236s after real Start). Distinguish
   the initial slowdown from later Recovery/penalty until contact is proved.

GNSS-derived map pose differs from EKF by roughly0.5m in globalY near the
boundary. These are different pose producers; neither is automatically a
simulator collider ground truth. GNSS median buffer is1 and simulation EKF
additional delay0. Its yaw is position-derived and stops updating at rest.
The diagnostic occupancy-cell-center check finds no occupied centers inside
either pose's declared wall-proof rectangle; this is not the native swept
cell-intersection proof and does not prove real-world clearance.

Recorded screen frame240s shows desktop wallpaper. It cannot corroborate the
vehicle/collider scene. The recorder grabs primaryScreen; do not mistake the
existence of MP4 for useful visual coverage. Unity Player.log is saved.
No production fix is selected. Next compare direct sensor/map geometry and
local simulator calibration; obtain actual collider evidence if needed.
