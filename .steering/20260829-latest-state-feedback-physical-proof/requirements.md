# Requirements

## Objective

Determine whether the latest-state steering feedback continuation can pass the
same current wall, timed dynamic-obstacle and Follow-gap proofs as a normally
reachable retained seven-state candidate.

## Frozen evidence

- Baseline commit: `4b1a980a`.
- Run `output/20260829-031339` projected 3148 of 3148 observed unreachable
  candidate evaluations and built 3076 exact nonlinear continuations.
- Domain 1 still exhausted its old executed cursor and entered canonical
  emergency while reconnectable candidates continued to arrive.

## Invariants

- Production continues to return `SteeringUnreachable` for every corrected
  shadow candidate.
- The corrected trajectory and a normally reachable trajectory use the same
  wall, dynamic-obstacle and Follow-gap proof code path.
- No clearance, solver tolerance, timeout, lease, grace, fallback, Mission or
  authority rule changes.
- No proof failure may be converted into authority.

## Exit criteria

- Unit tests distinguish nonlinear-only success from complete current-world
  physical proof success.
- Dynamic telemetry reports the complete proof acceptance rate and exact first
  rejection boundary.
- The next decision is production connector design or a return to the failed
  proof/model boundary; it is not a parameter adjustment.
