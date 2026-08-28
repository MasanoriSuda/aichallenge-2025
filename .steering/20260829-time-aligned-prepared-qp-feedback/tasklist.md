# Tasklist

- [x] Compare live compute cost with the upper GMPCC log.
- [x] Reject a second full-solve connector as the target architecture.
- [x] Record prepared-QP suffix contract.
- [x] Implement total stage-block and refinement-row slicing.
- [x] Add malformed provenance rejection tests.
- [x] Prove mixed-origin counterexample with a sliced feedback QP.
- [ ] Measure solve time against full semantic rebuild.
- [x] Run full build/tests.
- [ ] Decide whether runtime shadow is justified.

## Baseline evidence

- Current d1 full solve: about 100--110 ms average in heavy windows, maximum
  168 ms, often 3000--4000 iterations.
- Current d2 windows range from about 25 ms to 68 ms, with some 4000-iteration
  rejection windows.
- Upper `.steering/ano` main GMPCC: commonly about 22--35 ms while updating the
  current opponent relation continuously.

## Static result

- The old-final-QP/latest-x0 arm rejects the deterministic mixed-origin case.
- The absolute-time prepared-suffix arm solves the same case.
- Progress-wall, swept-wall and dynamic-obstacle rows advance with one common
  consumed-stage count; malformed old-stage provenance is rejected.
- `make autoware-build`: 25 packages passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 54 targets / 2065
  tests / zero failures.
- The small deterministic suffix solve completes in the test's 1 ms bucket,
  but this is not representative of the 20-stage live problem.  Production
  remains frozen until recorded 20-stage snapshots establish compute cost.
