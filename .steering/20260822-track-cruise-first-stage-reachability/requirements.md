# Track/Cruise first-stage reachability contract

## Baseline

- Branch: `develop_july`
- Baseline commit: `f954fdb`
- Preserve the unrelated user change `aichallenge/result-summary.json`.

## Observed phenomenon

After solved-progress course-frame alignment, `output/20260822-181304` produced 4,782 physical
certificates from 4,794 five-state solves. Candidate discrete hard contacts and course-frame
provenance failures were both zero. Two remaining candidate-side rejects were
`SweptPathViolation` at path index 1.

At decision 3767 near wp260:

- current production pose was individually clear;
- first solved MPCC pose was individually clear;
- conservative interpolation between those poses crossed an occupied wall cell;
- the solved first pose was about 1.16 m from the current pose with 0.232 rad yaw difference;
- about 50 ms later the legacy production pose was itself in hard wall contact.

The first diagnostic run also exposed a separate state-schema defect. The controller predicts the
control pose by `state_prediction_delay_sec=0.13`, but the five-state QP fixed initial `e_lag` to
zero at the nearest discrete waypoint. At decision 3808 in `output/20260822-192640`, projecting the
predicted pose into that waypoint frame produced `e_lag=-0.656 m`; the projected complete Frenet pose
reconstructed the control pose with zero reported position/yaw error.

## Competing hypotheses

### H1: True first-stage dynamic unreachability

The five-state QP endpoint is accepted by linearized Frenet dynamics, but the command-defined
continuous vehicle rollout from the measured pose intersects the wall. A stage endpoint constraint
alone therefore does not prove executable reachability.

Evidence: the legacy vehicle entered wall contact immediately afterwards.

Refutation: a physically integrated rollout using the solved first input is wall-clear and ends
within the accepted model residual of the solved first pose.

### H2: Certificate interpolation artefact

`evaluate_clear_footprint_path()` linearly interpolates world x/y/yaw between endpoints. A constant
curvature bicycle follows an arc, so the certificate may include impossible intermediate body
poses.

Evidence: both endpoints were clear and only an intermediate sample failed.

Refutation: a control-derived constant-curvature rollout also contacts the wall.

### H3: Current-state/course-frame projection mismatch

The QP state-zero hard equality uses `model->spatial_state`, while the certificate starts from
`actual_wall_monitor_pose_`. If converting the measured world pose into the solved course frame does
not reproduce state-zero `e_y/e_psi`, the QP begins from a different physical state.

Evidence: the first endpoint displacement and yaw change are large enough that this cannot be
excluded from the current log.

Refutation: state-zero reconstructed world pose matches the measured wall-monitor pose within an
explicit position/yaw tolerance.

## Required work

1. Add decision-scoped first-stage provenance: measured pose, reconstructed state-zero pose, solved
   first pose, first input, stage period, endpoint residual and both swept outcomes.
2. Add failure-first deterministic tests for constant-curvature integration and classification.
3. Run shadow authority and classify every path-index-1 failure as H1, H2 or H3 evidence.
4. Implement only the confirmed structural fix. Do not relax the exact footprint or wall map.

## Non-scope

- No Track/Cruise authority promotion.
- No wall-clearance, steering-rate, speed or solver-weight tuning.
- No Recovery or overtake behavior change.
- No acceptance of a connector solely because both endpoints are clear.

## Exit gate

- Every first-stage swept reject names the measured start, reconstructed state zero, solved endpoint,
  applied control, stage time and competing swept outcome.
- The selected correction is linked to one confirmed hypothesis.
- Physical certificate coverage is not increased by weakening collision geometry.
- `authority=shadow, selected=0` remains true.
- Build and complete package tests pass.

## Confirmed outcome

- H3 was partly confirmed as a schema problem: raw odometry and the state-zero pose differ by the
  intentional 0.13 s state prediction, but the predicted pose also has a real along-track residual
  relative to the discrete waypoint. That residual was discarded by the QP and wall certificate.
- H1 was confirmed for the remaining candidate rejects after the complete Frenet-pose correction.
  In `output/20260822-194818`, all three first-stage events were already in collision during the
  raw-to-predicted delay prefix; the control rollout began from a colliding pose at index zero.
- H2 was refuted for those events: a command-derived constant-curvature rollout did not clear the
  wall.
