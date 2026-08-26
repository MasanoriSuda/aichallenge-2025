# Design

This Slice is investigation-first. No wall, speed, solver, timeout, lease or
cost parameter was tuned to suppress the observed failure.

## Two distinct physical contracts

The investigation found two serial effects which must not be conflated.

1. AWSIM actuator transport maps the serialized command to the reported tire
   angle with an approximately 0.13 s delay and a static gain near 0.697.
2. The vehicle yaw response to that reported tire angle has an additional
   first-order response. A kinematic bicycle driven directly by the reported
   angle therefore leads the observed yaw response.

The first effect is an interface/unit contract. The second is vehicle dynamics.
Fixing only the interface makes desired, wire and measured steering consistent,
but does not make an instantaneous kinematic yaw model correct.

## Evidence

### Actuator wire to reported tire angle

| run | delay | measured / wire gain | R2 |
| --- | ---: | ---: | ---: |
| `output/20260826-182334` | 0.13 s | 0.698745 | 0.999165 |
| `output/20260826-184513` | 0.14 s | 0.695363 | 0.997595 |
| `output/20260826-192351` | 0.14 s | 0.696683 | 0.997481 |
| `output/20260826-193537` | 0.13 s | 0.6981 | 0.9975 |

The physical-equivalent command is serialized with the evidence-backed inverse
gain `1.435`. Model state, wall proof and retained revalidation remain in
physical-equivalent tire-angle units; the wire value never enters the bicycle
state.

### Reported tire angle to vehicle yaw response

The earlier first-order rejection applied to the actuator report itself. Once
the wire contract was corrected, four independent bags showed a separate yaw
response:

| run | pure-delay fit | gain | R2 | first-order tau | first-order gain |
| --- | ---: | ---: | ---: | ---: | ---: |
| `output/20260826-182334` | 0.120 s | 0.7885 | 0.9921 | 0.1388 s | 0.7865 |
| `output/20260826-184513` | 0.110 s | 0.7658 | 0.9923 | 0.1339 s | 0.7644 |
| `output/20260826-192351` | 0.120 s | 0.7368 | 0.9837 | 0.1462 s | 0.7401 |
| `output/20260826-193537` | 0.110 s | 0.7373 | 0.9889 | 0.1071 s | 0.7420 |

The adopted contract uses gain `0.75` and time constant `0.13 s`. These are
system-identification values, not lap-time tuning values.

## Selected model

The canonical normal formulation is now the seven-state temporal Frenet model:

```text
x = [e_y, e_lag, e_psi, v, progress, delta_command, delta_response]
u = [acceleration, delta_rate, progress_rate]

yaw_rate = yaw_response_gain * v * tan(delta_response) / wheelbase
delta_response_dot = (delta_command - delta_response) / tau
```

`delta_response` is inferred from measured yaw rate when speed makes the inverse
observable. At low speed the physical steering observation is used because yaw
inversion is ill-conditioned. Invalid or out-of-envelope inference closes
canonical normal authority; it is never clamped into feasibility.

The measured-to-control delay prefix and the QP use the same exponential yaw
response. The response state is explicitly bound into every Track, Cruise,
Follow, ShiftOut, Pass, Return, Rejoin and asynchronous pre-entry snapshot.
There is no zero-valued default path into production.

## Artifact compatibility

The formulation identity changed from the retired six-state name to
`VelocitySteeringYawResponseProgress7State`. State, bounds and cost schema IDs
were versioned to v2. A retained six-state artifact therefore cannot be joined
to a seven-state decision.

## One executable steering trajectory

The first seven-state Gates exposed a second producer-contract defect.  The QP
integrated its certified steering-rate sequence from the measured physical
steering state, while `ExecutionArtifact::extract_actuation()` integrated the
same rates from the previous desired command.  Replanning preserved and then
reintroduced this offset every cycle.  Consequently a wall-safe QP sequence
could publish a saturated command on another trajectory.

The seven-state formulation makes the two causal state roles explicit.  There
is exactly one executable command trajectory, plus one observed response
state:

```text
last successfully serialized physical-equivalent command
  + certified steering-rate sequence
  = QP command trajectory
  = wall-certified trajectory
  = physical-equivalent command trajectory

measured steering and yaw response
  -> response-state estimate at control origin
  -> vehicle yaw dynamics only
```

The previous serialized command is therefore the exact initial state of the
rate integrator and the retained command-reachability origin.  Measured tire
angle and yaw-derived response are not alternate command origins.  Re-basing
the integrated command on the delayed measurement each callback changes the
trajectory after it has been certified and creates an apparent phase lead or
lag.  Fresh and retained paths now apply the same command-state contract.

This corrects the interim single-origin interpretation inherited from the
six-state model.  In that model, binding steering to the observation avoided
using a desired command as an instantaneous yaw state.  Once response steering
is represented separately, retaining that binding conflates the command state
with the observed response and is no longer valid.

## Rejected alternatives

- wall margin, speed, OSQP, horizon or weight changes: downstream symptom
  tuning;
- holding measured steering across the full horizon: fixes neither yaw phase
  nor future curvature;
- keeping the six-state identity while silently changing dimensions: permits
  stale artifact adoption and makes logs untrustworthy;
- intersecting physical steering-rate prefix bounds with a previous desired
  origin: certifies two parallel trajectories but only wall-checks one;
- restoring a legacy normal controller: violates the single-authority target.

## Dynamic acceptance

Track/Cruise is the first Gate. The run must show bounded cross-track error,
no alternating steering growth, no delay-prefix/current-pose wall contact, no
Recovery, current seven-state formulation identity and sustained control rate.
Only after this Gate passes may Follow and Overtake dynamic acceptance resume.
