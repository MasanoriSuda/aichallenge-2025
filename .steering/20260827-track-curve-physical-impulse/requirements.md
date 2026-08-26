# Requirements

## Objective

Identify and repair the root producer of the repeated physical speed-collapse
near WP72--76 in `output/20260827-020001` without tuning wall margins, speed
weights, solver settings, or Recovery behavior.

## Observed invariant violation

A canonical Track/Cruise command which is solved, serialized and occupancy-map
certified repeatedly reaches an AWSIM physical event that almost stops the kart.
The external physical world represented by AWSIM and the world proven by the
controller are therefore not yet one execution contract.

## Scope

- correlate command, vehicle report, odometry, acceleration and IMU;
- compare every passage of the affected curve, including a user-provided
  upper-ranked bag;
- identify the earliest geometric or dynamic discriminator between clean and
  impacted passages;
- add a failure-first replay/test before changing production code;
- repair the producer contract and remove any downstream mask made obsolete;
- repeat Track/Cruise build, tests and six-lap Gate.

## Non-scope

- no wall-margin, speed, solver-weight or timeout tuning;
- no new normal authority, Recovery exception or legacy controller fallback;
- no inference that occupancy-map clearance proves AWSIM collider clearance;
- no Overtake performance tuning until Track/Cruise is durable.

## Timing contract

- A sensor `pose_additional_delay` may be enabled only from measured timestamp
  evidence; a 2025-derived provisional value is not a production default.
- Keep simulation and real-vehicle delay defaults independently selectable.
- Preserve the separately measured controller/actuator prediction horizon;
  this slice must not conflate sensor timing with actuator response.

## Acceptance

- the first physical impulse has a named upstream cause with recorded evidence;
- the production fix has a deterministic failure-first test;
- the launch contract rejects a nonzero default EKF simulation pose delay;
- canonical normal authority and one immutable execution fingerprint remain;
- six Track/Cruise laps have no abrupt external speed loss, wall event,
  Recovery cascade, legacy normal source, serialization rejection, or sustained
  callback overrun;
- generated result JSON and rosbag files remain uncommitted.
