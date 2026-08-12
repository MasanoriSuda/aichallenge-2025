# Tasklist

- [x] Confirm the mismatch between ContactContinuation/front-danger policy and
  committed Pass Behavior ownership.
- [x] Add bounded recoverable-contact authority to Pass geometry ownership.
- [x] Connect the existing runtime resolver output to the ownership request.
- [x] Test confirmed-overlap continuation and every hard-release boundary.
- [x] Build and test `multi_purpose_mpc_ros` in Docker.
- [x] Record `make dev2` acceptance criteria.

## Definition of Done

- Confirmed overlap owns Behavior only while the existing recoverable contact
  resolver is active.
- Non-recoverable contact and all shared hard faults remain fail closed.
- Package build and tests pass.

## Verification results

- `git diff --check`: passed.
- Docker `colcon build`: 25 packages passed.
- Focused committed-contact ownership tests: 3/3 passed.
- `multi_purpose_mpc_ros` package tests: 988 tests, 0 errors, 0 failures.
- Build stderr contained only the existing `setuptools` deprecation warning.

## Next `make dev2` acceptance criteria

- After `OvertakeLine ContactContinuation entered`, Behavior remains Overtake
  while the contact remains classified recoverable.
- No `V2X behavior: Overtake -> Follow` or SafetyBrake occurs solely because
  the same confirmed overlap is active.
- `committed Pass owns execution` is visible during that interval.
- The previous contact-driven speed collapse from roughly 4 m/s to 1.4--1.6
  m/s is absent unless a separate hard guard fires.
- Expired/stalled/high-energy contact still ends ContactContinuation and may
  enter SafetyBrake or Recovery.
- Wall-margin Recovery and solver recovery do not increase.
- Successful contacts proceed to `Pass -> Return -> Idle`.
