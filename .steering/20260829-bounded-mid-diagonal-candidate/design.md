# Design: bounded mid-horizon diagonal candidate

## Alternatives considered

1. Relax exact wall/obstacle proof: rejected because C/F already prove a
   physically valid candidate exists and the proof is detecting invalid
   geometry correctly.
2. Increase OSQP tolerances or SQP depth: rejected because A and the same
   stateless candidate fail while a different candidate succeeds.
3. Copy the exhaustive audit lattice into production: rejected because it is
   too expensive for the asynchronous live worker and obscures the bounded
   scheduling contract.
4. Replace the abrupt candidate with one normalized mid-horizon candidate:
   selected.  It adds the missing temporal homotopy without increasing the
   candidate count.

## Population

For the planning interval from the first valid dynamic-obstacle stage to the
terminal prediction stage `[first, terminal]`:

1. direct side;
2. physical diagonal from `first` to the integer midpoint of the interval;
3. late physical diagonal over the last third to the terminal stage.

The direct candidate already represents immediate avoidance, so retaining a
second two-stage immediate candidate is redundant.  The midpoint is a topology
sample, not a safety threshold; final feasibility remains owned by the
unchanged seven-state SQP and nonlinear certificates.

## Deleted path

Remove `EarliestPhysicalDiagonal` and its `first + 2` construction from the
production population.  Rename it to `MidPhysicalDiagonal` so logs and tests
state the actual represented homotopy.

## Falsifier

If the frozen production-right replay still fails after its candidate stages
match the certified audit candidate `(0, 9)`, candidate timing alone is not the
root cause and the production change must not be promoted.
