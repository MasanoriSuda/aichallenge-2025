# Requirements

## Objective

Explain and remove the stop/restart cycle observed in
`output/20260829-012053` without changing solver tolerances, wall clearance,
leases, grace periods, timeouts, or fallback policy.

## Frozen observation

- The seven-state worker continues to produce physically certified candidates.
- While the vehicle is moving, those fresh candidates are commonly rejected as
  `continuation-rejected` or `steering-unreachable`.
- The previously published artifact is then consumed until
  `progress-lift-rejected` / `cursor-unavailable`, which selects Stop.
- A fresh candidate becomes joinable again at cursor zero after the vehicle is
  nearly stationary.

## Root-cause gate

Before production behavior changes, retain candidate-specific current-world
join diagnostics even when the executed artifact wins selection.  The evidence
must distinguish cursor, progress, actuator, nonlinear-continuation, wall, and
dynamic-world rejection.

## Constraints

- Keep production authority unchanged during diagnosis.
- Do not tune physical or numerical limits.
- Do not add an age-only retained-plan exception.
- A candidate may become production authority only through a current-world
  physical connector from the last actually published command/state.
