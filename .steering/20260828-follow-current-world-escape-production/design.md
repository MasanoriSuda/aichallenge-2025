# Design

## Dataflow

```text
sealed Follow snapshot
  -> persistent Follow pipeline A
     -> certified: existing store path
     -> not certified:
          -> stateless positive-side seed -> private SQP -> proof
          -> stateless negative-side seed -> private SQP -> proof
          -> choose certified maximum terminal progress
          -> canonical candidate store
  -> next callback current-world retained revalidation
  -> canonical command
  -> serialized publication join
  -> mark executed
```

The worker is already latest-only, so a late population cannot queue an
unbounded backlog. Each side owns a fresh `SolverContext`; this is required
because Follow semantic identity intentionally remains side-free and therefore
cannot be used as cross-side warm-start provenance.

## Selection

Certified candidates dominate all incomplete candidates. Among certified
candidates, larger terminal progress wins; terminal velocity is a deterministic
tie-break. If neither side certifies, the most advanced diagnostic result is
returned and the existing Emergency supervisor remains the only wire owner.

The candidate builder remains data-only. Production authority is granted only
by the existing certified-plan store, current-world retained validator,
canonical command resolver and final actuation join.
