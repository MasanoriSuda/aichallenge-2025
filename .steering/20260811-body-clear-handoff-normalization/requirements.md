# Requirements

## Goal

Turn the bounded body-clear ownership handoff into a short, explicit transfer
to ordinary Pass ownership instead of a second authority that remains active
until a frozen deadline.

## Observed problem

In `output/20260811-212219/d1/autoware.log`, all 13 handoff entries already
reported `satisfied=1`; 10 nevertheless remained active until expiry and three
ended after current overlap was observed. The same run contained 14
`Overtake -> Follow` transitions, 25 Reverse requests, and 36 logged OSQP
failures.

## Scope

- Release handoff as soon as the ordinary Pass/front-cap latch owns execution.
- Shorten the frozen deadline with a live hard-gap TTC estimate.
- Preserve the Overtake Mission while preventing additional acceleration when
  the handoff is active but footprint prediction is invalid or non-separated.
- Log an explicit handoff release reason.

## Constraints

- Confirmed current overlap, target discontinuity, wall/path faults, emergency,
  and solver recovery remain hard guards.
- Do not change ROS 2 interfaces, launch files, YAML values, evaluation schema,
  or acceleration limits.
- Do not modify `aichallenge/result-summary.json`; it is user-generated output.
- Recovery redesign and asynchronous alternate planning are separate work.

## Acceptance criteria

- Ordinary Pass ownership ends the special handoff before its deadline.
- Live TTC can only shorten, never extend, the frozen deadline.
- Prediction uncertainty retains Mission ownership but applies a bounded speed
  reference instead of unrestricted Pass acceleration.
- Logs distinguish `normal_latch`, `expiry`, `current_overlap`, and hard/inactive
  release paths.
- Focused unit tests and `make autoware-build` pass.
