# Requirements

## Objective

Prevent a canonical Overtake plan from consuming the reserve needed to reuse
that same immutable plan across normal asynchronous control cycles.

## Repaired invariant

For ShiftOut/Pass/Return, every state sealed into an executable canonical plan
must remain inside the physical lateral corridor by the certified execution
tracking reserve.  Retained current-world proof continues to test the current
vehicle state against the unchanged physical corridor.

## Failure-first evidence

`output/20260824-135615/d1/autoware.log` first loses normal authority with:

```text
expected reserve = +0.067 m
measured reserve = -0.016 m
measured-to-expected lateral difference = 0.083 m
```

The configured `0.15 m` wall-tracking reserve is currently only a soft
reference adjustment; the solved trajectory is allowed to consume it.

## Constraints

- Do not change YAML parameters, solver iterations, tolerance, wall clearance,
  timing, lease, fallback, retry, grace, or phase transitions.
- Do not weaken current-world proof or Emergency authority.
- Do not alter Track/Cruise/Follow behavior in this Slice.
- Do not add a new normal command owner.
- Do not modify or commit `aichallenge/result-summary.json`.

## Definition of Done

- The contracted Overtake bounds are part of the solved QP.
- The required reserve is immutable plan provenance and plan validation rejects
  a nominal state that consumes it.
- The plan retains the original physical corridor for current-world proof.
- Existing soft-reference logic is not reinterpreted as a safety certificate.
- Focused tests, package tests, build and a bounded `make dev2` pass.
- Dynamic evidence shows either retained authority across the former boundary,
  or an earlier explicit `tracking-tube-unavailable` rejection without unsafe
  plan publication.
