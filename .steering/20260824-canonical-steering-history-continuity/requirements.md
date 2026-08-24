# Requirements

## Objective

Prevent an asynchronously solved canonical plan from publishing an actuation
whose first steering command is no longer reachable from the steering command
published by the current control history.

## Repaired invariant

Every canonical normal command must be certified against the current canonical
actuation history at selection time. A worker result being internally feasible
from its old snapshot is not sufficient after another plan has published.

## Failure-first evidence

Run `output/20260824-200419`, Domain 1, first Overtake episode:

- ShiftOut starts at ROS time `1787569501.344`.
- Published steering changes from `-0.09187` to `+0.00061 rad` in about 15 ms,
  then later from `-0.01687` to `-0.12303 rad` in about 41 ms.
- The configured first-cycle steering change is
  `steer_rate_max * Ts = 0.70 * 0.025 = 0.0175 rad` after steering-gain
  normalization.
- The worker that later fails still reports
  `previous_steering=-0.0956201`, proving that worker-local feasibility is not
  a certificate for the current publication history.
- The resulting oscillation moves the measured lateral state outside the
  retained corridor; Emergency braking and Stuck Recovery occur downstream.

## Constraints

- Do not tune steering rate, wall margin, Q/R weights, solver settings, timing,
  fallback, retry, lease, or grace.
- Do not raise Overtake phase/authority until Gate A includes current steering
  reachability; a rejected pre-entry plan must leave Track/Follow ownership
  unchanged.
- Do not clamp or mutate a certified command after selection.
- Do not add another normal authority.
- Apply one shared reachability contract to every canonical normal selection.
- Keep Emergency and Recovery as external overrides.
- Do not modify or commit `aichallenge/result-summary.json` or generated data.

## Definition of Done

- A pure contract rejects a finite canonical steering command outside the
  current one-cycle reachable interval.
- `CanonicalNormalSelection::complete()` requires this certificate.
- Fresh and retained Track/Cruise, Follow, Overtake and Rejoin selections use
  the same contract.
- The final publisher consumes the exact certified command without a new
  downstream clamp.
- Focused tests, full package tests and build pass.
- A bounded `make dev2` shows no uncertified canonical steering discontinuity.
- An unreachable pre-entry plan is rejected before `Idle -> ShiftOut`; it does
  not create a canonical-unavailable Emergency stop.
- An unavailable active-Mission replacement retains the existing certified
  plan.
