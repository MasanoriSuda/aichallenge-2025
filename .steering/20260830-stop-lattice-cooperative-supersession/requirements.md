# Requirements

## Objective

Prevent an obsolete observation-only Stop lattice epoch from exhausting the
complete candidate population while a newer immutable normal epoch waits.

## Root-cause statement

`LatestOnlyWorker` replaces a pending job but gives a running job no evidence
that its source has been superseded.  The Stop lattice evaluator therefore
continues solving all 68 candidates for an old world.  The pending newest
world cannot start until that obsolete evaluation returns.  This creates
multi-second result age even though the common certified Stop candidate now
solves in tens of milliseconds.

## Constraints

- Keep production authority unchanged.
- Keep candidate generation, solver settings and all proof policies unchanged.
- Do not add a Mission resume rule, lease, grace period, timeout or fallback.
- Do not interrupt a solver call or reuse a partially solved artifact.
- Supersession may be observed only at deterministic safe boundaries between
  candidate/proof operations.
- Preserve the existing non-cancelable `LatestOnlyWorker` API and behavior for
  all current production users.
- Record supersession as an explicit bounded observation result.

## Acceptance

- a generic worker token becomes superseded when a newer job is submitted;
- an old Stop evaluation exits after the current safe operation and does not
  start another candidate solve;
- a non-superseded evaluation retains the exact existing result;
- the observation mailbox remains monotonic;
- source authority audit proves no Store/publisher edge was added;
- static build/tests pass before `make dev2`;
- live evidence shows superseded observations and a lower hard-tail result age
  without production behavior changes.
