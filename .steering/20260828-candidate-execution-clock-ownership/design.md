# Design

## Root cause

The code had two lifecycle identities but one clock:

1. `candidate_plan_`: certified, never necessarily executed;
2. `executed_plan_`: its exact command crossed the publisher boundary;
3. `resolve_cursor()`: always used wall-clock time from the artifact prediction
   origin.

At cold start the asynchronous proof completed after the planned origin.  The
consumer sampled a later steering value as though earlier steering-rate inputs
had been published.  They had not: Emergency zero was the actual predecessor.
The reachability proof correctly rejected that fiction, but the next candidate
repeated it and created a permanent stop.

## Execution clock contract

Add an explicit `ExecutionClock` to current-world revalidation:

- `UnpublishedCandidate`: elapsed execution time is exactly zero;
- `PublishedPlan`: elapsed execution time is
  `current_control_origin - first_published_control_origin`.

The artifact prediction origin remains immutable proof provenance.  It is used
only as the coordinate origin passed to the existing artifact cursor resolver;
it no longer claims that an unpublished prefix was actuated.

The certified-plan store records `first_published_control_origin_sec` in the
same locked operation which records the published plan.  The publisher join
passes the control origin from the exact current-world proof.  Candidate
evaluation has no publication origin; retained evaluation consumes the atomic
executed snapshot.

## Safety argument

Starting a candidate at cursor zero does not reuse its old state trajectory.
The existing retained/current-world evaluator already:

- projects the live pose at the current control origin;
- replays the selected control suffix from that fresh state;
- checks publication-to-publication steering reachability;
- rebuilds exact nonlinear continuation;
- proves delay path, wall footprint, dynamic obstacles and Follow gap.

Thus this change removes fictitious execution history without weakening a
physical certificate.  If a candidate is too stale in pose/progress or its
cursor-zero controls no longer fit the world, the existing proofs reject it and
the actually executed plan remains available.
