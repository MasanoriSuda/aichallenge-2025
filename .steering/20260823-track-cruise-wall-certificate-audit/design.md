# Design

## Current causal boundary

The failed run had no V2X vehicle, no active overtake Mission, no Recovery
authority, and no callback overrun before the first incident. Fresh canonical
five-state MPCC solutions and their swept physical-wall certificates continued
to pass. AWSIM nevertheless reduced speed abruptly near waypoint 53 and the
wall monitor reported contact only after the incident.

The previously plausible actuator-scale hypothesis was falsified dynamically:
applying the legacy 1.5 gain to canonical commands moved the first incident
earlier and made steering reversals larger. That experiment has been removed.

## Competing hypotheses

### H1: nominal-plan certificate omits plant tracking uncertainty

The certificate proves the reconstructed nominal solution but does not prove a
tube containing the vehicle's reachable pose under measured lateral/heading
tracking error. **Rejected for the first incident:** immediately before the
speed collapse, failed-run cross-track error was approximately `-0.238 m`;
successful same-location crossings include approximately `-0.244 m`. The exact
static footprint sampler remained clear by at least its `1.0 m` search radius.

### H2: static wall grid and AWSIM collision geometry disagree

The exact physical footprint is clear in the static grid while an external
AWSIM collision/penalty source changes vehicle motion. **Supported at the
model-boundary level:** the runtime-equivalent sampler is clear by at least
`1.0 m`, yet speed falls from `10.089` to `1.424 m/s` in `0.120 s` under a
forward acceleration command. The exact AWSIM collider or penalty source is
not yet directly observable, so this evidence must not be overstated as a
specific wall-collider mismatch.

### H3: first-stage reachable motion differs from the reconstructed solution

The current-pose-to-stage-zero swept connector assumes a pose transition that
the published steering/vehicle dynamics cannot realize in one control stage.
**Not supported as the initiating event:** the measured steering follows the
published steering (`-0.0585` versus `-0.0631 rad`) and the controller does not
publish a braking command before the discontinuity. Connector quality remains
a separate canonical-authority concern, but it does not explain this measured
speed impulse.

## Evidence plan

1. Extract all odometry and control samples around the largest abrupt speed
   loss from failed and successful bags.
2. Project each pose onto the recorded runtime trajectory and report signed
   cross-track and heading error.
3. Compare the same waypoint range rather than comparing only elapsed time.
4. Inspect the certificate's exact clearance, connector, and stage geometry
   inputs.
5. If existing logs cannot distinguish H1-H3, add provenance telemetry only;
   do not modify control behavior in the same change.

## Selected correction

The root defect exposed by this incident is an incomplete evidence boundary:

1. the physical certificate proves only the configured static occupancy grid;
2. the controller expected `/aichallenge/pitstop/condition` to expose external
   collision state, but current AWSIM does not publish that topic; and
3. an abrupt measured speed loss therefore had no change-only event tying the
   command, pose, static-map observation, and external condition availability
   together.

Add a diagnostic-only speed-loss observer at the odometry boundary. It reports
only a consecutive-sample loss larger than both `1.0 m/s` and twice the
configured braking envelope, and records the current command and static-map
observation. It must never modify a decision, authority, trajectory, or command.
The absent condition topic remains in the development bag allowlist so an
environment that does publish it is captured without a code change.
