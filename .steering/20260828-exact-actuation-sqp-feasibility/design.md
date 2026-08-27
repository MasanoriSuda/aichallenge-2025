# Design

## Evidence

The physical-proof snapshot at sequence 1759 was solved with OSQP status
`solved inaccurate`. Its first acceleration is `1.376624 m/s^2` while the
physical upper bound is `1.37 m/s^2`. The adapter inset uses the base physical
row tolerance, but the generic certificate intentionally permits a larger
status-dependent tolerance for `solved inaccurate`. The artifact correctly
rejects the command; the producer and certificate do not share one contract.

The wall-refinement failures at sequences 1150 and 2342 reproduce with both
warm and cold starts. They therefore require an independent feasibility test,
not another warm-start rule.

## Approach

1. Add a read-only LP classifier for the serialized exact QP. It ignores the
   objective and asks whether all original `A`, `l`, `u` rows have any common
   solution.
2. Share the accepted solved-inaccurate residual multiplier between the solver
   certificate and the exact physical-bound inset. This changes only the
   solver-side interior margin; physical limits remain unchanged.
3. Add boundary tests proving that every accepted actuator value remains inside
   the original physical envelope even under the largest accepted row
   residual.
4. Use LP results to decide whether a further producer/model correction is
   justified. Do not modify production for an unexplained maximum-iteration
   snapshot.
