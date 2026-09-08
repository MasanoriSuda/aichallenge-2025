# Producer and acceptance

`racing_kart_gnss_poser::GNSSPoser` declares a finite, strictly positive
`unknown_position_covariance` in m². Only NavSatFix UNKNOWN uses it; known
diagonal covariance follows the existing path unchanged. Default is10.0.
`gnss_poser.launch.xml` exposes it, and simulation reference.launch supplies
the explicit local calibration0.1. Vehicle launch retains10.0.

The0.1m² value already denotes a good measurement in imu_gnss_poser's existing
quality mapping. It conservatively exceeds the local publisher's roughly
millimetre MGRS float/decimal quantization and1e-9-degree injected noise, and
allows heading-derived antenna lever-arm uncertainty. It is not inferred
from the all-zero UNKNOWN payload. Recalibrate when simulator/noise changes.

Regression publishes NavSatFix into the actual ROS node and inspects output
PoseWithCovarianceStamped: unknown calibrated input0.1, unknown default10,
known input preserved, invalid calibration rejected. Use local Cartesian
conversion and identical antenna/base frames to avoid unrelated geoid/TF
dependencies. The first calibrated regression must fail against old source.

Then build, package tests, inspect actual launched parameter and output, and
repeat bounded dynamic acceptance with fixed new binary/config. No new
normal command authority, command store, fallback or publication timer.
