# Audit

## Failure chronology

`output/20260823-065700` is not a Recovery-originated stop.  Ordinary Cruise was still owned by a
fresh-certified five-state solution when the vehicle hit the wall.  The speed/IMU discontinuity
precedes `SUSPECT_STUCK`, wall-contact corroboration and Recovery arbitration.

## Root contract conflict

- `multi_purpose_mpc_ros/README.md` defines `steering_tire_angle_gain_var` as AWSIM output
  compensation, not a bicycle-model curvature multiplier.
- `config/config.yaml` has used 1.5 since commit `490e1b2`.
- The authority-promotion implementation made canonical Track/Cruise bypass that calibration.
- The same run shows model-predicted curvature and measured curvature temporarily having opposite
  signs around rapid steering reversals.

These facts made the canonical model/actuator boundary a concrete hypothesis to test.  They did not
prove that the canonical command required the legacy multiplier; that conclusion depended on the
dynamic experiment below.

## Failure-first evidence

The execution-contract tests were changed before production code.  The focused target then failed
to compile because `CanonicalNormalCommand` had no model/actuator distinction, command construction
did not accept calibration, and publication still selected behavior with a boolean authority flag.
This is the expected failure for the identified contract gap rather than a runtime threshold
failure.

## Experimental implementation

- `CanonicalNormalCommand` temporarily stored `model_steering_tire_angle_rad` and
  `actuator_steering_tire_angle_rad` separately.
- Command construction temporarily accepted the existing positive finite calibration and failed closed on an
  invalid value or non-finite result.
- The controller used model steering for prediction/history and supplied the stored actuator angle
  explicitly to publication.
- The publisher serialized an explicit canonical actuator angle exactly; absent that value,
  legacy, canonical Emergency and Recovery retain their existing calibrated convention.
- Model-state mutation and serialized-actuator mutation were checked by separate contract functions.
- No gain, wall margin, speed, solver setting, retry, timeout, lease, fallback or ROS interface was
  changed.

## Static validation of the experiment

- Focused execution-contract binary: 49/49 tests passed after implementation.
- `make autoware-build`: 25 packages built successfully.
- Full `multi_purpose_mpc_ros` test target: all 37 CTest targets passed.
- `colcon test-result --verbose`: 1627 tests, 0 errors, 0 failures, 0 skipped.  The command also
  reports a pre-existing stale `joycon_contract_guard/package.xml` result-file warning; it does not
  affect the package result.
- Static search finds no old boolean physical-steering selector, old combined mutation checker or
  old canonical steering field reference.
- `git diff --check` passes.

## Dynamic falsification

`output/20260823-072038` started normally and did not reproduce the earlier wp53 event because it
failed sooner, during the first lap:

- at wp77: `raw=-0.1005`, `output=-0.1507`, measured curvature -0.19264 rad/m;
- at wp92: `raw=+0.0899`, `output=+0.1349`, measured curvature was still -0.08428 rad/m;
- at wp108: `raw=-0.0353`, `output=-0.0530`, measured curvature was still +0.11097 rad/m;
- within the next second actual speed fell from about 8.03 m/s to 0.37 m/s;
- Stuck Recovery then observed the left wall at 0.439 m and later actual map contact at wp113.

The collision predates Recovery.  The alternating command/response signs show that applying the
legacy multiplier to the already aggressive canonical closed loop worsens steering reversal rather
than repairing the plant contract.  This is dynamic evidence against the hypothesis, not a request
to tune the multiplier.

## Final disposition

All experimental production and test changes were removed.  Canonical normal and canonical
Emergency again publish the certified physical command angle exactly; legacy and Recovery retain
their existing calibrated convention.  Only this investigation record and the clarified canonical
exception in documentation remain.

The upstream cause of `output/20260823-065700` remains open.  It must be investigated independently
from steering calibration, beginning with actual tracking error versus the zero-margin physical
certificate around the first impact rather than adding another output transform.
