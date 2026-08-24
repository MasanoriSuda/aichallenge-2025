# Validation

## Root cause prevented

The established velocity-progress formulation uses both quadratic progress
tracking and a separate negative linear progress reward. The initial
rate-resolved QP request represented only references and weights, so a direct
runtime bridge would silently weaken the progress objective even though the
problem remained numerically solvable.

QP assembly now accepts an optional exact-size additional linear vector and
adds it before quadratic reference terms. Empty retains exact backward
compatibility. The semantic adapter maps legacy state and non-curvature input
linear terms. A nonzero independent legacy curvature linear term is rejected
rather than silently dropped or nonlinearly reinterpreted.

## Validation

- `make autoware-build`
  - PASS: 25 packages.
- Targeted QP/adapter tests
  - PASS: independent progress reward combines exactly with reference cost.
  - PASS: malformed dimension, non-finite coefficient and unsupported
    curvature linear term fail closed.
- Full `colcon test --packages-select multi_purpose_mpc_ros`
  - PASS: 43 CTest targets, 1,820 tests, zero errors/failures/skips.
  - Existing unrelated `joycon_contract_guard/package.xml` parser warning
    remains.
- Production link audit
  - PASS: no rate-resolved library is linked into `mpc_controller_cpp`.
- `git diff --check`
  - PASS.

## Next boundary

The six-state objective can now preserve current Track/Cruise semantics. The
next Slice may populate the adapter request inside the existing five-state
semantic builder and execute it in a latest-only worker. It remains shadow-only.
