# Requirements

## Objective

Classify and repair the first complete Pass failure from
`output/20260828-034416` without tuning solver, clearance, speed, lease,
timeout or fallback parameters.

## Frozen evidence

- production baseline: `e098f994`
- failure snapshot: sequence 2187, Pass, dynamic-obstacle refinement
- exact QP: mathematically infeasible with the production dynamic rows
- current physical homotopy: negative side of target

## Required invariant

Convexification around a wall-only witness may choose an obstacle disjunct,
but the obstacle-free witness may not erase a lateral homotopy which the
current physical state has already acquired.  Once an explicit Pass side has
full lateral body separation, every following valid stage remains on that
side until the supervisor changes the homotopy.  A separated middle sample
alone is not enough to establish that ownership.

## Out of scope

- no production authority or Emergency supervisor change;
- no threshold, margin, OSQP or controller-frequency tuning;
- no new Mission resume rule, lease, grace period, timeout or fallback;
- no claim of success until the refined trajectory passes the existing exact
  physical wall and dynamic-obstacle certificate.

## Definition of done

- same-snapshot LP classifies the late failure without promoting an
  uncertified partial escape;
- failure-first unit test covers the production geometry;
- dynamic-obstacle refinement preserves already acquired separation;
- existing exact physical certification remains unchanged and fail-closed;
- focused and package tests pass;
- moving Gate provides Pass/Return evidence or records the next earliest
  structural rejection.
