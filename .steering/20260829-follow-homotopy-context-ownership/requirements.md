# Requirements: Follow homotopy solver-context ownership

## Objective

Keep the numerical continuation state of each bounded Follow escape homotopy
across current-world submissions.  A certified side candidate must not be
followed by a cold solve merely because the population helper returned.

## Observed failure

In `output/20260829-095750/d1/autoware.log`, Follow candidate sequence 618 was
certified and published.  Its selected `follow-escape-negative` solve reported
`previous=empty-cache` and took 96.889 ms through the proof pipeline.  The
helper constructs a new `SolverContext` for every side on every call, so the
successful numerical state is destroyed immediately.

The normal worker then consumed only 5--7 results per roughly 80 submissions;
result age reached 0.3--0.6 s.  The executing plan later reached
`progress-lift-rejected`/`steering-unreachable`, leaving no certified normal
authority and causing Emergency Stop.  Earlier 20260829 runs exhibit the same
authority gaps, so this is not attributed to the current-world proof join.

## Constraints

- No timeout, lease, grace, fallback, solver tolerance or clearance change.
- Keep one canonical seven-state normal authority and the existing latest-only
  worker.
- Solver state may persist only inside a fixed Follow homotopy owner.
- Negative and positive candidates may not share primal/dual provenance.
- Track/Cruise, Overtake and publication contracts remain unchanged.
- Remove the per-call fresh-context path in the same Slice.

## Definition of done

- The normal worker owns primary, negative-Follow and positive-Follow solver
  contexts for its lifetime.
- Each candidate is evaluated only by the context matching its side.
- Tests prove no per-candidate context allocation remains.
- Dynamic logs show warm-start provenance per selected Follow side and permit a
  before/after comparison of compute age and authority gaps.
