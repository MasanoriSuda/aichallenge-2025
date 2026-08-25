# Validation

## Static checks

- Added seven focused tests for the steering-state contract. They cover a
  reachable committed input, positive and negative rate-limited motion,
  missing committed input, stale observation, and non-finite measurement or
  command.
- The focused test binary passes all seven cases.
- The full `multi_purpose_mpc_ros` package test run passes all 51 test targets
  (1863 tests, zero failures).
- `make autoware-build` passes all 25 packages. This build is the runtime
  workspace used by `make dev2`; a direct `/aichallenge/install` build alone
  is not accepted as runtime evidence.

## Dynamic check

Run: `output/20260826-044340` (`make dev2`)

- Both domains auto-started; no manual `/admin/awsim/start` request was used.
- Domain 2 logged the AWSIM `start` state and continued at approximately
  3.6--4.1 m/s.
- The new telemetry was present in both domains. For example, a command outside
  one latency prefix was resolved as measured `-0.1174`, origin `-0.2176`,
  committed `-0.2208`, reachable step `0.1001`, `command_reached=0`. The origin
  is therefore bounded between the physical observation and the already
  published input.
- Immediately before the first Domain 1 emergency stop, retained current-world
  validation accepted 79 of 80 observations. The physical origin was finite
  and the only steering rejection in that window was one candidate. The old
  measured-rate extrapolation was not the first failing invariant.

## Separate failure exposed by the run

Domain 1 later stopped during ShiftOut. The causal sequence is:

1. decision 1470 published a certified retained six-state command at about
   4.36 m/s;
2. a subsequent six-state solve reached 4000 iterations and was rejected;
3. the latest and last-feasible solved execution sources were both stale;
4. retained production proof became unavailable and decision 1478 published
   the canonical emergency brake at about 4.42 m/s;
5. after deceleration, progress-lift rejection and cursor exhaustion prevented
   normal authority recovery;
6. three stationary seconds later, Stuck Recovery started.

This is not repaired in this slice. It is a distinct solver/retained-source
continuity defect and must receive its own failure-first steering slice. No
timeout, solver setting, steering limit, fallback, or wall margin is changed
here to hide it.

## Result

The steering prediction-origin invariant passes static and dynamic validation.
The whole race is not accepted: Domain 1 exposes a separate production
continuity failure after a solve rejection. The containers were stopped after
evidence collection.
