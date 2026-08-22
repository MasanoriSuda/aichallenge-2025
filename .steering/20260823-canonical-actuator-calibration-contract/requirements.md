# Canonical actuator calibration contract

## Purpose

Test whether the first uninterrupted six-lap acceptance failure was caused by canonical
Track/Cruise bypassing the legacy AWSIM steering multiplier.  Preserve five-state MPCC as the sole
Track/Cruise normal authority and reject the hypothesis dynamically before retaining any code.

## Observed failure

Run `output/20260823-065700` completed lap 1 in 46.681 s, then collided near wp53 during ordinary
Cruise with no V2X vehicles.  Rosbag evidence at `1787435899.70` shows:

- vehicle speed changed from about 10.09 m/s to 1.39 m/s in one sample;
- IMU lateral acceleration reached about 1745 m/s2, proving an impact rather than commanded brake;
- the command still requested about 10.21 m/s, +1.37 m/s2 and -0.078 rad steering;
- the five-state solution remained fresh-certified and Recovery began only afterward.

The historical parameter contract defines `steering_tire_angle_gain_var=1.5` as legacy AWSIM output
calibration, while the authority-promotion Slice intentionally bypasses it for a certified
five-state canonical command.  The six-lap impact made that boundary a hypothesis to test, not a
root cause to assume.

## Constraints

- Do not restore Track/Cruise legacy MPC authority or add a runtime switch.
- Do not tune the gain, wall margin, speed, cost, OSQP settings, timeout, lease or retry.
- Preserve the existing configured steering calibration value.
- If explicit calibration is tested, represent model steering and serialized AWSIM actuator
  steering as different named fields and avoid anonymous publisher mutation.
- Preserve Recovery as an explicit whole-command supervisor override.
- Preserve ROS topic/message/launch contracts and the user-owned `aichallenge/result-summary.json`.

## Acceptance

- Failure-first tests isolate the proposed model/actuator distinction.
- Focused/full package tests and `make autoware-build` pass for the experiment.
- Dynamic evidence decides whether calibration is retained or fully removed.
- A rejected hypothesis leaves production source identical to the pre-experiment canonical
  publication contract and records why it was rejected.
