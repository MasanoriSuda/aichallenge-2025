# Requirements: canonical KKT scaling audit

## Objective

Determine whether one numerically canonical QP formulation/backend can solve
both the frozen ShiftOut failure `9845010060330222052` and the known Follow
counterexample at sequence 5575 without changing their objectives, physical
constraints or exact certificates.

## Constraints

- Production authority and normal solver remain frozen.
- No tolerance, iteration, clearance, timeout, lease, fallback or Mission
  change.
- No production retry among scaling variants.
- Every independently solved primal is unscaled to physical coordinates and
  must pass the unchanged exact C++ proof chain.
- A formulation that repairs one fingerprint and regresses the other is
  rejected as a canonical replacement.

## Definition of done

1. Compare current explicit scaling, raw physical coordinates, internal OSQP
   equilibration and available independent QP backends.
2. Pass candidate primals through the exact recorded affine and physical
   proofs.
3. Select one replacement candidate or classify the need for a different
   solver architecture.
4. Do not change production in this audit Slice.
