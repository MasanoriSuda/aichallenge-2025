# Dynamic validation

## Run

- controller logs: `output/20260823-170154/d1/autoware.log`,
  `output/20260823-170154/d2/autoware.log`
- duration: approximately six minutes after controller startup
- production config and authority: unchanged
- user result SHA-256 before/after:
  `03e2f3935d95a550d0e1a3f2006dde08dae4a7a4c74121c430b2452daf4414e6`

## Observation

This run contained no eligible Follow shadow window. Domain 1 repeatedly changed from Cruise directly
to `LowSpeedAvoidance` when it encountered the slower vehicle and then returned through Follow with no
current relevant/front observation. Domain 2 likewise had no coherent front vehicle.

Counts from the complete logs:

- Domain 1 `Follow MPCC shadow runtime` windows: 0
- Domain 2 `Follow MPCC shadow runtime` windows: 0
- Domain 1 V2X behavior transitions: 23
- Domain 2 V2X behavior transitions: 17
- observed positive encounter action: `Cruise -> LowSpeedAvoidance`
- observed Follow shadow event: `not-eligible/no-coherent-front-observation`

No QP was attempted for an invalid semantic Follow observation. This is correct admission behavior but
provides no evidence for or against live warm-start application.

Every observed Follow shadow event remained `authority=shadow, selected=0`. The user-owned result file
was unchanged. The isolated simulator result artifact was not emitted in this run, so lap/penalty data
is not used as evidence.

## Conclusion

The run is **inconclusive**, not failed. The test scenario did not exercise the acceptance boundary under
evaluation. Production parameters must not be changed merely to manufacture a passing rate and the
Follow authority gate remains closed.

The code-level causal fix remains supported by build, unit tests and the full package suite. The next
positive dynamic evidence must use a deterministic moving/stopped-front Follow scenario or a replay that
preserves the target observation and stage geometry. Until such evidence exists:

- do not promote Follow canonical authority;
- do not delete the scalar Follow owner;
- do not tune OSQP or distance/wall parameters;
- do not interpret successful `LowSpeedAvoidance` passes as Follow evidence.

## Isolated positive-Follow replay

The previous run's Domain 1 MCAP contains 229.16 seconds of recorded odometry, vehicle status,
trajectory and V2X input, including the positive Follow interval used for the baseline. It was replayed
to a new controller in ROS Domain 91 with no simulator or physical output connection.

Replay input:

```text
output/20260823-164329/d1/rosbag2_autoware/rosbag2_autoware_0.mcap
```

Only these input topics were replayed:

- `/v2x/vehicle_positions`
- `/vehicle/status/velocity_status`
- `/vehicle/status/steering_status`
- `/localization/kinematic_state`
- `/localization/acceleration`
- `/planning/scenario_planning/trajectory`

`/control/command/control_cmd` was deliberately excluded. `/awsim/state` was supplied only to reproduce
the recorded Grounded/Ready/Start controller session boundary.

### Stable Follow interval

Between replay Start and the recorded transition to `LowSpeedAvoidance`, five complete telemetry windows
contained:

| Boundary | Count |
|---|---:|
| valid contract / attempts | 90 |
| solved / normalized | 90 |
| physical certified | 90 |
| canonical plan/cursor/candidate/authority/actuation/command | 90 at every boundary |
| canonical-ready shadow | 90 |
| warm starts applied | 90 |
| solver context resets | 0 |
| row rejects | 0 |

Timing for those 90 cycles:

- build: 0.036 ms average, 0.061 ms maximum
- solve: 0.890 ms average, 1.432 ms maximum
- certificate/canonical chain: 0.464 ms average, 0.757 ms maximum
- total Follow shadow: 1.420 ms average, 2.276 ms maximum

Every result remained `authority=shadow, selected=0`.

### Replay limitations

The full replay contained admission transitions and cold-start intervals that do not form one stationary
comparison cohort. Across all 364 attempts it recorded 257 accepted results, 246 applied warm starts and
12 intentional context resets. After one below-hard-gap contract interval, cold solve failures occurred
before the problem became solvable again. Those intervals must not be averaged with the stable positive
Follow cohort to claim a race-level improvement.

The recorded V2X source header age appears degraded during replay even though receipt age and observation
generation are coherent. The full Autoware stack and bag player also shared host CPU, so callback-overrun
telemetry is not representative of a live AWSIM run. Neither limitation affects the narrow proof that the
complete Follow dual is shifted, applied by OSQP, physically certified and converted through the exact
canonical chain.

## Updated conclusion

The structural warm-start repair passes its dynamic replay gate: the former `warm=0` boundary became
`warm=90/90` in the stable Follow interval, with 90/90 canonical-ready results and zero downstream reject.

This is sufficient to accept the warm-start integration fix. It is not sufficient to promote Follow to
production authority because the positive proof is replay, not a live moving-front run. The scalar Follow
owner remains in production until a live eligible interval confirms the same chain under real callback
timing.
