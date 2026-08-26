# Audit

## Observed symptom

V2X-empty Track/Cruise runs reached a wall although a canonical normal artifact
had passed solver and wall certification. QP iteration failures and Recovery
started after contact and were downstream symptoms.

## Causal chain

```text
wire/physical steering unit mismatch
  -> corrected actuator calibration exposes the next error
instantaneous kinematic yaw assumption
  -> predicted turn response leads measured vehicle response
  -> corrected by the explicit yaw-response state
seven-state state-role mismatch inherited from the six-state repair
  -> the steering-rate integrator is re-based on delayed measured steering
  -> the publisher still serializes that integrated state as a command
  -> each callback changes the command trajectory's causal origin
  -> the published trajectory is not the wall-certified trajectory
  -> steering reversals and saturation are issued against the wrong phase
  -> cross-track oscillation grows through consecutive curves
  -> delay-prefix wall proof becomes blocked after avoidance is unavoidable
  -> current-pose contact
  -> QP/Recovery cascade
```

## Competing hypotheses

- Wall-map-only failure is rejected because current-pose footprint monitoring
  later observed the same physical contact side.
- Callback overload is rejected as initiator; sustained solver failure started
  after contact.
- A variable actuator plant is rejected by the delayed static wire-to-report
  fits with R2 above 0.997.
- The immediate full-gain kinematic yaw model is rejected by four independent
  report-to-yaw fits with gain about 0.74--0.79 and 0.11--0.15 s response.

## Root cause

The prior six-state formulation represented commanded/reported steering as the
state which immediately generated yaw. AWSIM instead exhibits a distinct yaw
response state. Consequently the same command was certified in one dynamic
model and executed by another. Downstream wall checks were internally
consistent with the wrong predicted trajectory.

After adding that state, a second root cause remained: the rate-integrated
steering state was still initialized from delayed physical observation, an
invariant that had been appropriate only while steering also represented yaw
response.  In the seven-state model that state is the serialized command and
its input is steering rate.  Re-basing it on observation each cycle violated
the state equation and made solver, retained proof and publisher disagree
about the causal command predecessor.

The curvature-reference gain correction exposed and amplified the oscillation,
but it is not the root producer defect: the state-role inconsistency exists
even at unit gain.  Emergency braking and Recovery masked the source by
appearing after the wall trajectory had already become unavoidable.

The detection gap was a contract test that asserted only that one steering
origin existed.  It did not assert that the selected origin belonged to the
state input/output semantics.  A new failure-first test now requires the
command state to originate at the last serialized command and the response
state to originate independently from observation.

## Repair

- preserve measured physical, physical-equivalent desired and actuator-wire
  steering as separate meanings;
- infer a response-steering state from odometry yaw rate;
- predict the transport prefix with the identified response model;
- add response steering as a seventh QP state;
- bind that state and the identified gain/time constant into every producer;
- version formulation and schema identity so retired artifacts cannot survive;
- reject unavailable/unphysical response observations instead of clamping.
- make the last successfully serialized physical-equivalent command the sole
  steering-rate integration and command-extraction origin;
- keep measured/yaw-derived response steering as the independent yaw-dynamics
  initial state;
- make retained steering reachability use the previous serialized command and
  its actual publication age;
- remove the observation value from all command-origin roles.

## Static verification

- `make autoware-build`: passed.
- full `multi_purpose_mpc_ros` test suite after the state-role repair:
  46/46 CTest targets, zero failures.
- rate-resolved model, adapter, problem, shadow, retained, physical proof,
  command candidate and single-authority tests all passed.

## Remaining proof

`output/20260827-010414` confirmed exact serialization joins and removed the
previous command/wire mismatch. Its first later authority alternation was not
a recurrence of the steering-state defect: investigation in
`20260827-receding-prefix-authority` identified whole-suffix retained
revalidation as a separate execution-boundary defect. A complete six-lap
Track/Cruise acceptance remains required after that repair. Static success and
the initial clean lap prove contract closure, not sufficient race durability.

`output/20260827-020001` subsequently completed six recorded laps and retained
exact command serialization, but failed the durability Gate because a
physical speed-collapse recurred near WP72--76. The first drop began while the
serialized command was still +1.366 m/s2 and before Emergency authority. This
does not reopen the command/response ownership finding; it starts a separate
physical path/collider investigation.
