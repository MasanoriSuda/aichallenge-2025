# Requirements

## Objective

Make current-world Follow proof construct its dynamic-target tube over the
exact remaining time domain of the retained canonical plan. A relinearized
current MPCC horizon must not make an otherwise current target observation
disappear at the terminal retained stage.

## Failure-first evidence

Dynamic run `output/20260824-222801`, Domain 1, contains 12 steady-state
`canonical-follow-emergency` cycles:

- 6 `target-horizon-unavailable` rejections;
- 4 physical `stage-gap-violation` rejections;
- 2 steering-continuity rejections.

All six target-horizon failures reject both the incoming and retained plan in
the same cycle. Five reject at retained stage 19 and one at stage 8. Their
minimum gaps remain 4.20--6.43 m while the hard gap is 2.05 m. The target is
current and identified; only the finite time vector ends before the retained
window sample.

The current Follow contract and retained plan derive stage durations from
separately relinearized horizons. Equal state counts therefore do not imply an
equal cumulative time domain. Reusing the current solver's target vector as if
it necessarily covered a previous plan's remaining time is the broken
assumption.

## Constraints

- Do not change Follow distance, wall margin, steering rate, solver settings,
  timeout, lease, grace, or fallback policy.
- Do not accept a stale target or weaken target identity/tube fingerprints.
- Use the current observation's same constant-velocity prediction model to
  cover the exact retained execution window.
- Preserve `stage-gap-violation`; a covered horizon that predicts a hard-gap
  breach must still fail closed.
- Keep the operation pure and testable; do not add controller-only special
  cases.
- Keep `aichallenge/result-summary.json` untouched and uncommitted.

## Definition of Done

- Target horizon coverage is explicitly constructed from a current target
  observation and a required retained-window terminal time.
- The covered observation is re-fingerprinted and remains tied to the current
  observation generation.
- Unit tests cover already-covered, moving-target extension, stationary-target
  extension, malformed input, and preserved hard-gap rejection.
- Package tests and `make autoware-build` pass.
- A bounded dynamic run has zero `target-horizon-unavailable` Follow authority
  losses, while physical gap violations remain visible if they occur.
