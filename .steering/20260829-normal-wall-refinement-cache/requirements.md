# Requirements: canonical wall-refinement evidence cache

## Objective

Remove repeated immutable static-wall scans from the canonical normal MPCC
worker's critical path without changing any candidate, constraint, tolerance,
certificate or production-authority rule.

## Frozen evidence

In `output/20260829-115006/d1/autoware.log`, Follow result `seq=710` was
captured at `17.980 s` for control origin `18.110 s`, but was consumed around
control time `18.560 s`.  The selected single-side solve was valid and used a
warm start, yet the pipeline checked 1,883 static-wall poses and took about
200 ms.  At adoption the steering delta was `0.427877 rad` while only
`0.019833 rad` was reachable.  The old executed plan then exhausted and normal
authority fell to Emergency Stop.

The defect is therefore not a clearance threshold.  A result whose physical
proof was valid became unusable because immutable wall evidence was rebuilt
inside every receding solve and the asynchronous result missed its causal
control origin.

## Constraints

- Keep the current seven-state formulation, candidates, solver settings,
  physical clearances and exact final wall/dynamic proof unchanged.
- Cache only immutable clear-run evidence for an exact quantized stage-wall
  geometry. A cached search range may serve only an equal or narrower query;
  a wider query must rescan the union and replace the evidence.
- Cache identity must include the immutable occupancy-grid fingerprint and
  every geometric input that affects occupancy. Query bounds are represented
  by the entry's explicitly certified search range, not by the key.
- A hash collision must never reuse evidence: exact key equality is mandatory.
- Cache lifetime is one side-specific `SolverContext`; it owns no Mission,
  Store, command or publisher authority.
- Remove no final proof and add no fallback, timeout, lease or grace period.

## Definition of Done

1. Repeating an identical or narrower wall refinement performs no additional
   footprint scan and returns proof-equivalent geometric bounds.
2. Changing grid identity, footprint, pose bucket or sampling settings
   produces a miss; expanding lateral bounds rescans the union.
3. Cache size is bounded and deterministic eviction cannot affect acceptance.
4. Existing wall-refinement and full package tests pass unchanged.
5. A bounded `make dev2` run reports cache hits and compares worker compute,
   result age and authority gaps with `output/20260829-115006`.
