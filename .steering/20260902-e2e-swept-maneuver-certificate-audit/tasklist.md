# Tasklist

- [x] Freeze the offline certificate scope and physical assumptions.
- [x] Implement pure kinematic rollout and footprint clearance.
- [x] Add synthetic geometry and contract tests.
- [x] Implement sequential bag replay with teacher-side comparison.
- [x] Run successful-train, successful-validation and failed-peer cases.
- [x] Classify before dataset or production changes.
- [x] Keep production authority and checkpoints unchanged.

## Result

Rejected as a discriminator and label source.  A current-scan swept-footprint
rollout does not isolate the failed run from either successful run.  Temporal
dynamic occupancy remains necessary.
