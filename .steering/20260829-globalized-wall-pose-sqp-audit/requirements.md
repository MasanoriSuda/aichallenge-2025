# Requirements: globalized wall-pose SQP audit

## Objective

Determine whether the existing bounded proof-guided SQP can be extended from
"a QP solved but exact proof failed" to the earlier case where the first
post-hoc lag/heading wall trust region has no feasible affine step.

## Constraints

- Production authority, Store, command publication and configuration remain
  unchanged.
- No clearance, tolerance, lease, timeout, fallback or iteration-budget
  tuning.
- A pose-bucket-relaxed QP is observation-only and cannot certify itself.
- Exact trajectory, physical wall, current-world obstacle and terminal
  successor proofs remain mandatory.
- Compare against frozen fingerprint `11478535197026802675` before designing
  a production replacement.

## Definition of done

1. The audit can omit lag and heading pose buckets together without altering
   lateral/progress wall rows, dynamics, actuators or obstacle rows.
2. The unchanged proof chain classifies the resulting racing iterate.
3. If the first iterate fails exact proof, a bounded proof-guided relinearized
   arm is evaluated before any production change.
4. The result is registered as an architecture decision.
