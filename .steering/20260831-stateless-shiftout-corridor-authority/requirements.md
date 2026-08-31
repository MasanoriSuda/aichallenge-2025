# Requirements: stateless ShiftOut corridor authority

## Objective

Prevent a publisher-certified stateless sibling ShiftOut from being aborted
solely because the legacy gap/Mission candidate search cannot regenerate the
same corridor in a later control cycle.

## Frozen evidence

Use `output/20260831-162051/d1/autoware.log`, especially the first sibling
adoption at lines 1356--1371.

## Constraints

- do not change wall clearance, solver tolerance, timeout, lease or grace;
- do not bypass wall, target, emergency, continuity or solver hard faults;
- do not retain an unpublished Mission path;
- only an exact publisher-bound stateless source may supersede the legacy gap
  planner's entry-candidate verdict;
- keep one normal command publisher and immutable artifact identity.

## Acceptance

- the same-snapshot publisher-certified sibling remains normal ShiftOut
  authority when a later legacy candidate search reports no gap;
- a raw planner failure still blocks an uncommitted ShiftOut;
- current wall/target/emergency/solver faults still abort normally;
- dynamic `make dev2` no longer enters Recovery with
  `live overtake corridor unavailable` immediately after sibling adoption.
