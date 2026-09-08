# Explicit calibration for GNSS with unknown covariance

Preserve NavSatFix topic/type and known-covariance behavior. The local AWSIM
publisher supplies covariance_type=UNKNOWN, all zeros, despite declared
position noise1e-9 degrees and delay0. GNSSPoser invents10m²; imu_gnss_poser
maps this to100m². The EKF consequently receives position standard deviation
10m. Same-run source250.599994/receive1788798709.4679 proves this chain.

Replace the producer's hardcoded fallback with a validated explicit sensor
calibration. Default10m² preserves uncalibrated/vehicle callers. The simulation
launch supplies0.1m² for this local AWSIM sensor model. This is a conservative
measurement variance, not a physical error bound or a 2026 official value.
Do not alter known covariance, EKF gains/process model/delay, gyro data,
controller margins, or simulator noise to obtain a pass.

The same-seed consumer comparison at240s uses two actual EKF binaries with
identical recorded pose/twist and timing. At250.605s, changing only position
variance100→0.1 changes y43169.6645→43169.1254; reconstructed body y43169.0679.
Original hidden EKF history is unavailable. This is causal contribution
evidence, not an exact race replay or proof of avoiding the collision.

The actual wall mesh projects into free map cells and physical body extents
also differ. Those independent geometry defects remain in the physical-stop
audit. Sensor calibration alone cannot establish integrated safety.

Rollback baseline: a8b968ac plus the preserved fixed-binary trial3 patch.
Restore only this slice's GNSS/launch changes when rolling it back.
