# Design

## Audit chain

```text
Gate-A proposal
  -> Mission freeze / DP execution source
  -> Idle -> ShiftOut
  -> six-state atomic transition solve
  -> CertifiedPlan store
  -> retained current-world revalidation
  -> final normal command
```

For each boundary, join decision, Mission generation, target generation, side,
source timestamp, problem/solution/plan identity and physical proof result.

## Initial hypotheses

1. The frozen DP source is valid at admission but expires before a refreshed
   source can replace it; the six-state command then loses required semantic
   path input.
2. A valid six-state CertifiedPlan exists, but retained current-world proof
   invalidates it after DP authority loss; Emergency is therefore downstream.
3. The callback overrun delays or starves refresh/consumption and is causal,
   rather than merely concurrent.
4. Target stale/lost is later than source loss and is a symptom of the episode,
   not the first defect.

No hypothesis authorizes a fix until log and source ownership agree.

## Confirmed causal chain

The source audit found that `record_solved_mpcc_execution_trajectory()` had
no caller.  That retired helper accepted a legacy `MpcProblem` primal, while
the production six-state pipeline stored an immutable `CertifiedPlan` and
never projected that exact physical trajectory into the rolling ShiftOut
execution source.

```text
six-state Gate A accepted
  -> exact six-state CertifiedPlan published and stored
  -> no six-state CertifiedPlan -> rolling source edge
  -> entry-time DP source reaches its 0.5 s freshness boundary
  -> no safe solved source can replace it
  -> DP authority released
  -> retained normal proof unavailable
  -> explicit Emergency
  -> target loss and Recovery occur later
```

The callback overruns and target loss are downstream or independent.  They do
not precede the missing source edge.

## Selected correction

A pure adapter consumes only an exact six-state `CertifiedPlan` and matching
intent, target, Mission generation and side.  It projects the certified
physical path distance, lateral position and course progress into the existing
rolling source schema.  This projection is not a command authority and cannot
reinterpret a five-state primal.

The original artifact observation timestamp is retained.  Re-adoption cannot
renew source age, and only a newer artifact sequence can replace the current
projection.  The obsolete legacy recorder is deleted in the same change.

## Separate defect discovered after repair

Both the pre-change run `output/20260825-231050` and the post-change run
`output/20260825-233538` show the same later six-state ShiftOut solver collapse.
For `N=20`, `failed_iterate_row=254` decodes to the stage-zero
virtual-progress-speed input box.  This is not caused by the new projection:
it existed before the projection was connected.  Its formulation and
first-stage feasibility require a separate root-cause Slice; this Slice does
not tune OSQP or add a fallback to hide it.
