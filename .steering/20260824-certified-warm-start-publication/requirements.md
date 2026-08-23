# Requirements: certified warm-start publication

## Objective

Stop a raw five-state OSQP success from becoming the next horizon's warm-start
authority before semantic and physical certification has accepted it.

## Constraints

- Do not change MPC/MPCC weights, bounds, tolerances, wall margins, timeouts or
  fallback policy.
- Keep warm start as a numerical hint only; it must never confer execution
  authority.
- Apply one lifecycle contract to Track/Cruise, Follow, Overtake and isolated
  left/right branch contexts.
- Preserve user-owned `aichallenge/result-summary.json`.

## Acceptance

- A solver-successful but downstream-rejected artifact is not reusable.
- Only a semantic/physical accepted artifact is published to warm history.
- Publication stores the normalized execution primal rather than the raw
  near-bound OSQP primal.
- Unit tests prove single-use consume, stale rejection and malformed publish
  rejection.
- Package tests and build pass.
- Bounded runtime evidence no longer shows a persistent warm-only reject chain.
