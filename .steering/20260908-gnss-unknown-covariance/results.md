# Local calibration accepted; integrated geometry acceptance still open

Before change, the ROS pub/sub test requested0.1m² for UNKNOWN but observed
10m² on all three axes. After change, four ROS behavior tests pass: calibrated
UNKNOWN, unchanged10m² default, preserved known covariance, invalid values.
The package-wide checks pass22tests/0errors/0failures/0skips, including
cppcheck2.7 explicitly enabled with Google Test library definitions. Initial
lint failures (missing test copyright, missing macro definitions) were repaired.
Final `make autoware-build`:25packages pass; log is preserved with this run.

Diagnostic `20260908-gnss-calibration-single-r1`: same controller binary
5b65133ad69c424ff3c1954cc73b778b9d9aef283aff0d05586ac90f6ef0ae96,
same map/controller settings, changed GNSS producer and simulation launch.
Six laps248.777923584s,penalty0; laps43.875168/40.484444/41.119579/
40.984550/40.954544/41.359631s. Formal resultv3 supplies six laps even though
controller's seam log counts five before AWSIM finish.

No active-race moving Emergency or Recovery; moving Emergency11172/11173
occurs after AWSIM finish1788801379.988733414. Callbackmax18.804ms,
0overruns across13093reported cycles. Control receive10444messages over
261.070666s,mean24.999585ms,p9526.310849,p9927.077136,max33.563137ms,
0gaps>50ms and0nonpositive receive gaps. Source stamps have34nonpositive
gaps and max115ms; those are separately recorded, not confused with receive.

All recorded EKF position inputs are0.1m² instead of the failed run's100m².
The same analysis interpolates EKF XY to raw sensor time, reconstructs body
XY from GNSS/IMU and the inspected extrinsics, and selects source>20s,
speed>1m/s,odom gap<=50ms. These are different closed-loop realizations.

| Metric | Failed trial3 | Calibrated diagnostic |
|---|---:|---:|
| Moving samples |4611|4866|
| XY error mean |0.201967m|0.033200m|
| XY error p95 |0.353457m|0.067862m|
| XY error max |0.698433m|0.251212m|
| Absolute lateral error p95 |0.296189m|0.050570m|
| Absolute lateral error max |0.488128m|0.112663m|
| Prior contact region XY error p95 |0.488616m|0.065619m|

Sensor reconstruction retains float/decimal quantization and is not an
independent motion-capture system. The paired same-input EKF comparison is
separate causal evidence. One clean diagnostic is not repeated acceptance.
Map coverage, physical body enclosure, dev2/Stop/async/dev3/dev4/gates and
submission-image acceptance remain open under the parent completion plan.
