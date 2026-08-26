# Design

## Evidence order

1. Separate the first physical impulse from Emergency and Recovery.
2. Cluster all impulses by world pose and course passage.
3. Compare clean and impacted passages for speed, yaw, lateral position,
   steering response and acceleration.
4. Compare an upper-ranked run through the same world region.
5. Inspect the exact MPCC prediction/certificate that produced the impacted
   command.
6. Only then select a production repair.

## Root-cause result

### H1: longitudinal arbitration initiated the stop

Rejected for the first event: the preceding wire command requested
+1.366 m/s2. Emergency and `velocity-unreachable` followed the measured loss.

### H2: localization alone jumped

Rejected as the complete symptom explanation: the raw vehicle velocity report
falls before the filtered odometry, localization acceleration records the
impulse, and one recurrence contains an IMU spike over 900 m/s2.

### H3: AWSIM Wall penalty creates the observed 5 km/h plateau

Confirmed. The v3 result contains seven `wall` events and no `crash` or `over`
event. The shipped AWSIM IL clamps horizontal rigid-body speed to exactly
1.388889 m/s while any penalty is active. Event timing and the observed
plateau agree.

### H4: the map and physical wall checks use different time origins

Confirmed as the upstream producer. `reference.launch.xml` gives EKF an
unmeasured 0.3 s simulation pose delay. The source design explicitly called it
a provisional 2025 value without timing evidence. The controller then applies
the separate 0.13 s measured actuator prediction. At 8--9 m/s, the EKF/raw-GNSS
displacement around the impacted curve reaches roughly 2--2.4 m, so MPCC's map
certificate does not describe the AWSIM rigid-body pose being penalized.

## Structural repair

Set the simulation EKF additional-delay default to zero while keeping the
launch override and real-vehicle default. Retain the 0.13 s controller horizon:
it is a different, measured actuation contract used by the seven-state model.

Add a source contract test which requires both EKF delay defaults to be zero
and verifies that the selected override is still passed to EKF. This removes a
past workaround instead of adding a margin, speed cap, waypoint exception, or
fallback.
