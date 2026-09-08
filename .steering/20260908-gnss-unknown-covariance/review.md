# Review

Producer uses explicit positive finite covariance only when the incoming
NavSatFix declares UNKNOWN. It does not mistake the zero payload for certainty.
Known diagonal behavior and uncalibrated10m² default remain unchanged. The
new simulation branch is selected by the existing simulation launch argument;
the vehicle branch keeps its default and no vehicle process was started.

No EKF tuning, sensor-noise reduction, control/certificate relaxation, new
command path, held authority, or delay correction is introduced. The0.1m²
calibration is local to the inspected AWSIM model and is explicitly documented
as statistical, requiring recalibration when that model changes.

The regression exercises the actual ROS producer, not a duplicated formula.
Explicit cppcheck registration retains full package analysis and loads the
Google Test macro model; no source files or diagnostics are suppressed.
Build, package checks, XML branches and the closed-loop covariance are verified.

Remaining risks: position-derived GNSS yaw/lever-arm error, IMU extrinsic
conventions, simulator covariance metadata remaining UNKNOWN upstream, and
independent physical world geometry gaps. This slice does not resolve them.
