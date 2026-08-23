# Requirements

## Objective

Determine whether the exact five-state wall rejection isolated in
`output/20260824-063046` is a true predicted collision or an artifact of
linearly connecting sparse world-frame stage poses through a tight curve.

## Scope

- Preserve the exact rejected interpolated pose and segment fraction.
- Compare the current world-chord sweep with a course-frame/Frenet-resampled
  sweep from the same five-state solution.
- Change production proof only if a failure-first test and dynamic evidence
  show that the current chord does not represent the solved trajectory.

## Constraints

- No wall-margin, solver-tolerance, weight or horizon tuning.
- No fallback, retry, lease, cooldown or feature flag.
- No weakening of footprint sampling or fail-closed behavior.
- Preserve the user's `aichallenge/result-summary.json` change.

## Acceptance

- A swept failure reports the actual rejected interpolated pose, segment and
  segment ratio, not the safe endpoint.
- Curved-course replay distinguishes world-chord and Frenet/course-frame sweep.
- Any production change is tied to that evidence and retains conservative
  sub-grid footprint sampling.
