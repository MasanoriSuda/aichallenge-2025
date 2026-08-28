# Results

## Frozen input

- source decision: `2473`, intent `ShiftOut`
- original failure: `wall-refinement-solve-rejected`
- restored final QP snapshot:
  `000000002473-shiftout-wall-refinement-restored-solve-rejected`
- source interaction fingerprint: `11398499380521175557`

Production authority and all configuration values remained unchanged.

## Observations

1. The original full wall-refinement QP is affine-infeasible. Removing only
   refinement-owned heading buckets makes its affine constraints feasible.
2. The audit-only Phase-I projection solved three bounded SQP tangent updates
   and rebuilt fresh full physical wall buckets.
3. The rebuilt final full QP is exactly feasible. HiGHS found a solution with
   maximum physical row violation `1.11231e-13`.
4. Production OSQP reached 4000 iterations on that same final QP with primal
   residual `0.000264417` and dual residual `0.0172868`. It therefore created
   no artifact; no acceptance tolerance was weakened.
5. The independent HiGHS primal was rechecked by C++ arm I against the exact
   recorded QP. Its maximum row-normalized violation was `1.11231e-10`.
6. The unchanged physical adapter, nonlinear execution trajectory, swept wall
   proof, all-obstacle dynamic proof and terminal-successor proof all accepted.
   Arm I produced a complete audit-only ManeuverBundle with candidate
   fingerprint `4811741141782047813`.

## Classification

This frozen failure is **not physical infeasibility** and is **not a
model/certificate mismatch**.

It is the composition of two defects:

1. the one-shot wall bucket freezes heading/lag/progress around a provisional
   wall-unsafe tangent and creates an affine-infeasible QP;
2. after feasibility restoration rebuilds a valid final QP, the current OSQP
   backend does not converge to the optimum within its existing contract.

The first defect is a feasibility-restoration/SQP construction problem. The
second is a backend/convergence mismatch. Treating either as wall-margin,
clearance, timeout, lease, fallback or generic solver-tolerance tuning would
hide the cause.

## Production recommendation

Do not connect either audit arm to production yet. Open a separate Slice that
compares production-available QP backends or a bounded feasible-QP/globalized
SQP implementation under the existing 25 ms scheduling contract. Promotion
requires the same complete ManeuverBundle proof, runtime p95/p99 evidence and
atomic fallback to the last actually published certified artifact.

The production change, if accepted later, should replace the failed wall
refinement/solve mechanism rather than add another Mission resume rule,
clearance exception or OSQP tolerance override.

## Verification

- `make autoware-build`: passed, 25 packages.
- `colcon test --packages-select multi_purpose_mpc_ros`: 52/52 CTest targets,
  2056 tests, zero failures.
- frozen restoration arm: Phase-I solved three times; final production OSQP
  correctly rejected at its current convergence contract.
- external HiGHS solve: optimal, 307 QP ASM iterations, about 0.01 s reported
  solver time.
- external-primal exact proof arm: `stage=accepted`, `bundle=1`.

## Remaining risks

- One frozen ShiftOut failure is classified; other Pass/Return/Stop snapshots
  may have different causes.
- CasADi/HiGHS is an offline audit dependency, not a production integration.
- The reported HiGHS kernel time does not establish end-to-end 40 Hz runtime,
  deterministic memory behavior or production packaging compatibility.
