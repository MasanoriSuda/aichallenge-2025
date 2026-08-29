# Design: proof-guided SQP acceptance

## Acceptance contract

For one sealed candidate:

1. solve and certify the current single-SQP formulation (`depth=0`);
2. if certified, return it and do not run another iterate;
3. otherwise evaluate fresh deterministic audit arms at depths 1, 2, and 3;
4. after each depth, run the unchanged exact proof chain;
5. return the first certified ManeuverBundle;
6. if none certify, report the strongest rejection evidence but create no
   executable artifact.

This is an outer nonlinear-solver acceptance rule, not a control fallback.
Only a fully certified bundle can win.  The production worker is not changed
in this Slice.

## API boundary

The audit `SolverContext` accepts an explicit bounded depth (1--3).  Normal
`evaluate()` always passes zero.  The architecture comparator owns depth
enumeration and proof evaluation because it already owns immutable replay
world certification and has no publisher API.

## Evidence and interpretation

The frozen corpus separated two earlier hypotheses:

- a later iterate is not universally better: an already certified candidate
  must be retained at depth 0;
- one additional physical-consistent linearization can repair a marginal
  nonlinear proof mismatch without changing homotopy, bounds or clearance.

The two recovered candidates both certified at depth 1.  Candidates that were
solver-infeasible stayed rejected, and one strongly nonlinear opposite-side
candidate stayed dynamically rejected.  This supports conditional iteration
after exact rejection, not unconditional fixed-depth SQP and not further
clearance/tolerance tuning.
