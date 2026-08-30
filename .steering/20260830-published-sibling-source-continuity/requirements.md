# Requirements: published sibling source continuity

## Objective

Eliminate the publication-ledger split observed after an opposite-side
current-world Bundle crosses the sole command publisher.  The next control
cycle must consume the latest actually published source identity without
claiming that a stateless Bundle's unmodified source plan was executed.

## Frozen evidence

`output/20260830-160431/d1/autoware.log` records the following causal chain:

1. decision 2363 publishes ShiftOut source sequence 1700 on side `-1`;
2. the publication token atomically commits the tactical side `1 -> -1`;
3. the next-cycle execution alignment reads exact-executed sequence 1639,
   which still belongs to side `1`;
4. alignment rejects it as `side-mismatch`;
5. normal authority is lost shortly afterwards and Stop/Recovery follows.

## Constraints

- Do not promote a stateless current-world Bundle's unmodified source plan to
  exact executed evidence.
- Do not add a lease, grace period, timeout, fallback, retry, solver setting,
  wall margin or clearance change.
- Keep the sole publisher boundary and existing sibling token revalidation.
- Choose the latest published source under one Store lock; consumers may not
  independently race the exact-executed and Bundle-source ledgers.
- Preserve target, Mission generation, intent, side and publication-cursor
  checks.

## Definition of Done

- Store exposes one atomic latest-published-source snapshot with explicit
  provenance (`exact-executed` or `current-world-bundle`).
- published Overtake alignment consumes that snapshot.
- source-contract and unit tests cover Bundle-over-executed precedence and
  later exact-publication supersession.
- build and package tests pass.
- dev2 observes opposite-side adoption without immediate published
  `side-mismatch`, or the run records that the adoption scenario did not occur.
