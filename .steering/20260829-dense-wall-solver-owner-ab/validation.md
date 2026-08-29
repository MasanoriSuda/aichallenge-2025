# Validation: dense-wall solver-owner A/B

## Isolation

The two arms rebuild the same dense nonlinear interior-wall problem from one
immutable snapshot and receive the same warm primal, SQP budget, OSQP
iteration limit and physical proof chain.  Each
`LatestStateFeedbackSolverContext` owns exactly one solver policy selected at
construction:

- A: `row-normalized`;
- B: `row-normalized-internal-equilibration`.

B is not a retry after A and neither context has a Store, mailbox or publisher
API.  The existing single-authority source contract initially rejected a
two-workspace context; the implementation was corrected to one preselected
owner per context rather than weakening the contract.

## Frozen replay at 25 ms

### ShiftOut sequence 1266

- both arms assemble 336 dense interior-wall rows;
- A reaches 4000 iterations in about 59.3 ms and produces no artifact;
- B reaches 4000 iterations in about 59.4 ms and produces no artifact;
- classification: `dense-solver-owner-not-causal`.

Internal equilibration does not resolve the frozen ShiftOut failure.

### Follow sequence 531

- both arms assemble 478 dense interior-wall rows and pass the unchanged exact
  physical proof;
- A: 2650 iterations, about 45.2 ms;
- B: 450 iterations, about 8.5 ms;
- classification: `both-dense-arms-accepted`.

Conditioning is materially relevant to runtime for this feasible Follow case,
but it does not distinguish feasibility because both owners certify it.

### Cruise sequence 601

- both arms assemble 177 dense interior-wall rows;
- A reaches 4000 iterations in about 48.4 ms and produces no artifact;
- B reaches 4000 iterations in about 48.7 ms and produces no artifact;
- classification: `dense-solver-owner-not-causal`.

Cruise remains an independent unresolved QP case.

## Decision

Do not promote internal equilibration to the latest-state production owner in
this Slice.  It is a useful runtime optimization on the feasible Follow
snapshot, but it is not the root cause of the remaining ShiftOut or Cruise
failure and therefore cannot justify a production solver-policy change.

The exit condition from the prior Slice is met: stop OSQP-local scaling,
iteration-limit and row-topology work.  The next comparison must use an
independent structure-exploiting or nonlinear feasibility oracle on the exact
same frozen ShiftOut and Cruise problems.  Production authority and parameters
remain frozen until that oracle classifies them as candidate-generation,
single-SQP or physical-infeasibility defects.

## Verification

- focused latest-state tests: 55/55 passed;
- single-authority source contract: 75/75 passed;
- `make autoware-build`: 25 packages passed;
- frozen ShiftOut, Follow and Cruise replay: completed;
- full package regression: 54/54 targets, 2,084 tests, zero errors,
  failures or skips;
- `git diff --check`: passed.
