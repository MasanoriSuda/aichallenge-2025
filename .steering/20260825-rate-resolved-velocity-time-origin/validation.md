# Validation

## Observed phenomenon

In `output/20260825-081954`, retained rate-resolved candidates frequently
failed as `velocity-unreachable` even though their speed change was feasible
over the controller's measured-to-control delay prefix.

| Run/domain | Attempted | Accepted | Dynamic blocked | Steering | Velocity |
|---|---:|---:|---:|---:|---:|
| old `081954` d1 | 404 | 236 | 17 | 5 | 137 |
| new `084721` d1 | 404 | 382 | 17 | 1 | 0 |
| old `081954` d2 | 4,692 | 4,403 | 99 | 0 | 178 |
| new `084721` d2 | 3,317 | 3,212 | 95 | 0 | 0 |

The unequal d2 attempt totals reflect different run durations; the new run
still accumulated 3,317 attempts without one velocity rejection.

## Root cause and causal chain

`current_speed_mps` is measured at callback observation time. The retained
actuation is sampled at `control_origin_sec`, about 130 ms later in the runtime
configuration. The validator used the 25 ms publication period to calculate
the reachable speed interval anyway:

```text
measured velocity at observation
  -> compared to predicted velocity at control origin
  -> allowed change for only one publication interval
  -> physically reachable candidate rejected as velocity-unreachable
  -> retained six-state command unavailable
```

Steering does not share this defect: its predecessor is the last committed
published steering command, so its reachability interval remains one command
publication period.

## Failure-first evidence

`UsesObservationToControlDurationForVelocityReachability` creates a 50 ms
observation-to-control prefix and a `2.00 -> 2.05 m/s` candidate. The old
implementation rejected it as `VelocityUnreachable`, because `a_max=1.0`
permits only `0.025 m/s` over the incorrectly used 25 ms period. The corrected
implementation accepts it using the exact 50 ms prefix. The independent
`RejectsUnreachableVelocity` test still rejects a truly unreachable value.

## Implemented change

- velocity reachability uses `control_origin_sec - now_sec`;
- steering reachability still uses `publication_interval_sec`;
- no additional publication interval is added;
- result diagnostics preserve steering/velocity deltas, velocity bounds and
  velocity duration even when proof construction rejects;
- runtime telemetry prints those values for rejected and accepted results.

No parameter, solver option, cadence, fallback or authority was changed.

## Static validation

- Failure-first focused test: failed before implementation as expected.
- Corrected focused tests: 2/2 passed.
- `make autoware-build`: 25 packages passed.
- `multi_purpose_mpc_ros`: 49/49 test targets passed.
- `colcon test-result --verbose`: 1,891 tests, 0 errors, 0 failures.
- `git diff --check`: passed.

`colcon test-result` also emitted the pre-existing stale
`build/joycon_contract_guard/package.xml` lookup warning. Its final result was
still zero errors/failures; no generated workspace state was edited to hide
it.

## Dynamic validation

Command and artifact:

```text
make dev2
output/20260825-084721
```

Both domains logged `velocity_duration:0.130000` and finite reachable velocity
bounds. Velocity rejects fell to zero. Legitimate `dynamic-path-blocked`
results remained visible, including d1 `blocked_by:d2` with negative signed
clearance. All command candidates remained
`formulation:velocity-steering-progress-6state`,
`authority=shadow, selected=0`. The inspected run accumulated 3,901 d1 and
3,891 d2 callback cycles with zero overruns.

## Decision and remaining risk

This Slice is accepted. It repairs a producer/consumer time-domain mismatch
without weakening obstacle or wall admission.

Production promotion is still blocked. A retained suffix that is
`dynamic-path-blocked` must not be reused, and the current shadow has no
same-formulation fresh current-world command for every such cycle. The next
Slice must define the atomic six-state fresh/retained admission boundary and
the handling of genuinely blocked current paths before deleting the five-state
Track/Cruise owner.
