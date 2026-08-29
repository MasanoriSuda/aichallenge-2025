# Requirements: current-world proof join

## Objective

Make one immutable current-world certificate evaluate the exact trajectory
against both the static wall and all observed dynamic obstacles before an
Overtake candidate may enter the certified-plan store.

## Root cause

The current production pipeline stores a plan after solver and exact wall
acceptance.  Exact dynamic-obstacle clearance is deferred to retained
revalidation.  A candidate can therefore become the latest certified plan and
only then be rejected by the publication join.  This separates candidate
selection from the physical fact that should decide it.

## Constraints

- No parameter, tolerance, lease, timeout, fallback, or authority change.
- Use the same immutable replay world, physical footprint, control prefix,
  course-frame knots, exact trajectory and swept sampling contract.
- First extract and test the shared proof.  Production wiring follows only
  after architecture replay and unit tests are equivalent.
- A dynamic rejection cannot create or replace a certified plan.
- Existing retained revalidation remains the publication-time proof; the new
  proof closes the earlier candidate-admission gap.

## Definition of done

- World-pose reconstruction has one shared implementation.
- Architecture comparison uses the shared exact dynamic proof.
- Production Overtake candidate evaluation requires the joined wall/dynamic
  certificate before store replacement.
- Proof-guided SQP can use the joined certificate without an audit-only code
  path.
- Frozen replay, focused tests, full build and a dynamic run are recorded.
