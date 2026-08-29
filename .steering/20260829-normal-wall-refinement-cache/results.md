# Results: canonical wall-refinement evidence cache

## Verification

- `make autoware-build`: passed.
- `multi_purpose_mpc_ros`: 54/54 CTest cases passed; 2,084 assertions,
  zero errors and zero failures.
- Dynamic run: `output/20260829-122448` (`make dev2`).

## Dynamic result

The covering-range cache produced real reuse while preserving the unchanged
exact proof chain. Representative accepted results reported:

- `cache_hits=20/cache_misses=0/cache_scanned=0`;
- `cache_hits=16/cache_misses=4/cache_scanned=385`;
- `cache_hits=19/cache_misses=1/cache_scanned=48`.

Cold or changed geometry still produced misses and full scans. This is the
required failure-closed behaviour: the cache never creates a safety
certificate and an absent covering entry always returns to the original scan.
Several steady Cruise periods completed in roughly 18--31 ms, while changing
geometry and hard QP cases remained slower. The cache is therefore accepted
as a bounded removal of duplicated immutable raster work, not as a cure for
all worker latency.

## Falsified root-cause claim

The d1 vehicle still lost normal authority during ShiftOut. Snapshot
`000000000639-9f4bf2f22099dbf3-shiftout-wall-refinement-solve-rejected`
captured the earliest relevant failure. The wall-refined racing QP reached
OSQP's iteration limit and its exact affine set is empty with both post-hoc
lag and heading buckets present.

Independent replay established:

- full recorded affine QP: infeasible;
- omit lag bucket: HiGHS, ProxQP and qpOASES solve the racing QP;
- omit heading bucket: the same independent backends solve it;
- current A/B/C candidate families still fail before producing a certified
  bundle.

Consequently, repeated wall scanning was a measurable scheduling cost but not
the root cause of this ShiftOut stop. The remaining blocker is the post-hoc
wall-pose bucket formulation and its interaction with the seven-state
linearization/certificate. No timeout, lease, clearance or solver tolerance
was changed in this Slice.

## Decision

Keep the proof-equivalent cache because it removes duplicated static work and
has bounded ownership. Do not use its presence to classify an authority gap
as solved. The next Slice is an architecture escape-hatch audit of the same
immutable snapshot: exact physical proof of bucket-relaxed independent
solutions followed by an offline nonlinear/multi-SQP feasibility arm.
