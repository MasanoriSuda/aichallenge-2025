# Requirements

## Objective

Classify the frozen Return authority failure at decision 773 without changing
production authority, solver settings, clearance, leases, timeouts or fallback.

The failure must be separated into one of:

- persistent Mission lifecycle / exhausted horizon;
- candidate generation;
- single-SQP or numerical solver limitation;
- model / physical certificate mismatch;
- physical infeasibility.

## Frozen evidence

- Run: `output/20260828-162503`
- Snapshot:
  `d1/mpcc_architecture_snapshots/000000000773-return-wall-refinement-solve-rejected/snapshot.yaml`
- Production transition: `Pass -> Return` succeeded initially.
- The rejected Return request later contained only 4 stages, while the other
  frozen ShiftOut request contained 20 stages.
- Every existing A--G architecture arm stopped at solver rejection, row 63.

## Constraints

- Do not change production command authority.
- Do not change OSQP tolerance, iteration count, weights or scaling.
- Do not change wall, opponent or tracking clearances.
- Do not add a Mission resume rule, lease, grace period, timeout or fallback.
- A conclusion must be reproducible from the immutable snapshot.

## Acceptance

- Decode the failed row semantically.
- Independently test feasibility of the exact recorded linear constraints.
- Compare the current horizon policy with the local upper-rank log and primary
  MPCC references.
- State the earliest violated invariant before proposing a production change.
- Preserve the configured temporal horizon unless a separately certified
  lateral-tracking prefix intentionally shortens it.
