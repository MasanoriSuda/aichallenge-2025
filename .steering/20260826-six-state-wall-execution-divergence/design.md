# Design

## Initial causal hypotheses

1. **Command transport mismatch**: the certified six-state steering sample is
   not the steering command that reaches AWSIM.
2. **Actuation/model mismatch**: the kinematic six-state prediction assumes
   `yaw_rate = v * tan(delta) / wheelbase`, while AWSIM produces substantially
   less curvature in the high-speed hairpin.
3. **Time-origin mismatch**: wall proof begins from a predicted delayed state
   but publication or retained cursor uses a different observation/control
   origin.
4. **Course-coordinate mismatch**: the proof and actual pose use different
   progress branches or Frenet frames near the hairpin.
5. **Insufficient current-state envelope**: the retained proof validates the
   nominal path but does not reject growing measured tracking error before the
   nominal path itself remains wall-clear.

## Investigation order

1. Decode rosbag topic availability and time bases.
2. Join control command, kinematic state, steering status and IMU/yaw rate over
   `1787669555.5--1787669559.1`.
3. Reconstruct commanded kinematic curvature and measured curvature.
4. Trace the wall certificate's initial state, delay compensation and retained
   current-state checks in source.
5. Select a correction only after the first mismatch is proven.

## Non-solutions

- increasing wall margin to hide an unmodelled response;
- lowering speed before proving the mismatch owner;
- accepting an older plan for longer;
- cold retry or relaxed OSQP tolerance;
- special-casing waypoint 115--124.

## Evidence and causal conclusion

### Joined incident timeline

The first wall incident in `output/20260825-235153` occurs around
`1787669558.9--1787669559.0`.

- At `1787669557.639`, the six-state solve and physical wall proof are both
  accepted and the published steering demand is approximately `0.335 rad`.
- Between `1787669557.6` and `1787669558.2`, the published demand rises from
  about `0.326 rad` to `0.358 rad`, while `/vehicle/status/steering_status`
  reports only about `0.205--0.255 rad`.
- The command-derived kinematic curvature is about `0.32 1/m`; the measured
  yaw-rate/speed curvature is only about `0.11 1/m` at the start of the
  divergence.
- At `1787669558.966`, the actual footprint reaches `wall_distance=0.000` and
  `e_y=-2.426 m` while the behavior is still Cruise.
- The sustained six-state row rejection cascade occurs after this contact.

The command topic contains the same steering demand selected by canonical
production, so command transport is not the first divergence.  The solve was
healthy before contact, so solver failure is also downstream rather than the
cause of this incident.

### Source data flow

The sixth state is a physical steering-angle state: its dynamics integrate the
steering-rate input and the vehicle model uses that state to predict curvature.
However, every production submission currently binds
`Request.current_steering_rad` from `previous_steering`, which is the last
published desired angle.  The current-world retained request repeats the same
substitution.  The controller does not subscribe to
`/vehicle/status/steering_status`.

Consequently one desired command is carrying two incompatible meanings:

1. predecessor command for publisher continuity; and
2. observed physical steering state for vehicle prediction and wall proof.

The physical wall proof then certifies the nominal path of the desired-angle
state, not the path reachable from the measured actuator state.  Retained
revalidation cannot expose the mismatch because it is initialized by the same
desired command.

### Root cause

The earliest proven contract defect is **command/physical-state conflation at
the six-state prediction origin**.  It overstates available curvature while
the vehicle is entering the hairpin, so a nominally wall-clear horizon is not
an execution certificate for the actual vehicle.

The correction must establish one observed steering-state producer, project
that observation onto the existing control prediction origin, and bind every
fresh, transition, pre-entry and retained six-state path to that value.  The
last command remains a separate value used only for publisher continuity.

This correction does not claim that the configured steering-rate model is
fully calibrated.  Once state provenance is correct, any remaining difference
between predicted and measured steering response is a visible model
calibration defect rather than a hidden state-identity defect.
