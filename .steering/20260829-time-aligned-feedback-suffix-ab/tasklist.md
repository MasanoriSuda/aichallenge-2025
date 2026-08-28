# Tasklist

- [x] Freeze production authority and mixed-origin regression.
- [x] Record causal contract and promotion gate.
- [x] Implement pure elapsed-stage resolution.
- [x] Implement complete semantic suffix rebuild.
- [x] Add boundary, exhausted-horizon and dynamic-stage alignment tests.
- [x] Prove deterministic old-QP failure / suffix success.
- [x] Run package build and full tests.
- [ ] Add observation-only runtime A/B.
- [ ] Run bounded `make dev2` and record rates/timing/proof outcomes.
- [ ] Decide AS-RTI suffix vs current-state low-rate GMPCC.

## Static result

- Arm A (`x0` replacement in the old final QP) rejects the narrow-progress
  counterexample.
- Arm B consumes two elapsed stages, rebuilds the one-stage semantic suffix and
  solves the same physical state without changing bounds or solver settings.
- A partial-stage fixture shifts state/input, nominal path, obstacle stages and
  forced transition indices with one common clock.
- A remaining stage shorter than the model's declared minimum is rejected; it
  is not rounded, merged or hidden by a grace period.

## Verification

- `make autoware-build`: 25 packages completed successfully.
- Direct A/B gtest filter: 4/4 passed.
- Full `multi_purpose_mpc_ros` test suite: 54 targets, 2062 tests, zero
  errors/failures/skips.
