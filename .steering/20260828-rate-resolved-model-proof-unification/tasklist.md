# Tasklist: Rate-resolved model/proof unification

- [x] Reproduce snapshot 890 with production-equivalent offline replay.
- [x] Prove a feasible command sequence exists for the frozen constraints.
- [x] Identify the coarse-SQP versus dense-proof transition mismatch.
- [x] Implement one canonical nonlinear transition.
- [x] Delete the physical adapter's duplicate transition implementation.
- [x] Add transition/tangent/equivalence tests.
- [x] Run focused tests and full package tests.
- [x] Replay the frozen failure and run dev2.
- [x] Update the audit.
- [x] Commit the structural Slice.

## Dynamic follow-up (not part of this root-cause repair)

- [ ] Preserve the concrete execution-artifact rejection reason instead of
      reporting a default physical-proof result.
- [ ] Classify the Pass -> SafetyBrake interruption in run
      `output/20260828-022111` from its final decision authority.
