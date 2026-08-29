# Task list

- [x] Freeze the current ShiftOut failure snapshot.
- [x] Confirm the full recorded affine QP is infeasible.
- [x] Confirm independent backends solve each one-bucket-relaxed racing QP.
- [x] Add an observation-only external-primal bucket oracle.
- [x] Add exact-mode and physical-proof-chain regression tests.
- [x] Run focused tests and build.
- [x] Run both independent bucket primals through exact proof.
- [x] Run an exact-dynamics physical oracle through the unchanged proof chain.
- [x] Record classification and retain the audit-only evidence path.

Decision: retain the audit tool and do not promote a bucket bypass.  The
frozen candidate is physically feasible, but neither one-bucket affine solve
is an executable proof.  Production work must replace the one-shot hard
bucket formulation with proof-guided/globalized successive convexification;
it must not weaken wall clearance or accept an uncertified QP iterate.
