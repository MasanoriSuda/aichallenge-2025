# Audit

## Evidence boundary

- Baseline: `f0fe5c8`
- Instrumented source: uncommitted observation-only Slice based on that baseline
- Runtime: `output/20260824-022828`
- Domain 1 / Domain 2
- AWSIM admin start subscriber was unavailable. The dynamic evidence is a
  bounded pre-race closed-loop run, not a lap-performance evaluation.

## Expected and observed behavior

Expected:

```text
OSQP solved
-> every physical row is inside its own convergence tolerance
-> semantic execution-primal certificate accepts
-> one canonical command may execute
```

Observed on Domain 2:

```text
OSQP solved with pri_res approximately 0.003--0.018
-> common adapter admits against a global tolerance approximately 0.015--0.019
-> acceleration / predicted velocity / virtual progress speed exceeds its
   own approximately 0.001--0.005 row tolerance
-> execution-primal-reject
-> canonical Emergency Stop for that cycle
```

The bounded run contained 31 certified and 34 execution-primal-rejected
Track/Cruise outcomes. Rejected semantic rows were:

| Field | Count | Mean violation | Mean row tolerance | Mean ratio |
|---|---:|---:|---:|---:|
| acceleration | 16 | 0.005355 | 0.002375 | 2.25 |
| predicted velocity | 17 | 0.007341 | 0.005174 | 1.42 |
| virtual progress speed | 1 | 0.001398 | 0.001001 | 1.40 |

Representative decisions are 545, 553 and 853 in
`output/20260824-022828/d2/autoware.log`.

## Earliest violated invariant

The earliest violated invariant is:

> A solver result advertised as successful for canonical execution must be
> certified under the same unit-aware constraint contract consumed by the
> execution adapter.

The producer is the common OSQP adapter's mixed-unit global post-solve
acceptance. It computes one tolerance from the largest physical constraint
scale. In this five-state problem, course-progress values are approximately
14--18 m while acceleration, speed and curvature rows use much smaller units.
OSQP 0.6.2 is configured with `eps_abs=1e-3`, `eps_rel=1e-3`, ten internal
scaling iterations and `scaled_termination=0`.

## Root cause, contributor, detection gap and mask

- **Root cause:** the five-state mixed-unit QP has no coherent
  nondimensionalized convergence contract. A single unscaled/global residual
  tolerance is allowed to certify rows with incompatible physical units.
- **Contributor:** acceleration and velocity objectives frequently place their
  optimum at an active upper/lower bound. Ordinary ADMM residual then crosses
  the small-unit physical boundary more often; this changes frequency, not the
  underlying contract mismatch.
- **Falsified hypothesis:** warm-start stage transport is not the root cause.
  `.steering/20260824-stage-aligned-warm-start-transport` observed the same
  rejects with zero-stage and cold starts.
- **Detection gap:** Track/Cruise checks lateral rows and executable state/input
  boxes, but does not require every dynamics/rate/progress-wall row to satisfy
  its row tolerance before certification.
- **Downstream detector:** `normalize_extended_execution_primal()` correctly
  rejects executable acceleration, curvature, virtual-progress and predicted
  velocity bound violations.
- **Mask/recovery:** retained-plan revalidation often cannot be used in the
  observed pre-race world, so the correctly rejected fresh solve becomes an
  Emergency Stop. Emergency Stop is not the root cause.

## Additional safety finding

For `N=20`, rows 270--289 decode as curvature-rate constraints. The largest
normalized violation in many outcomes was row 270--273. Even the 31 outcomes
currently labelled `certified` had an average maximum normalized row violation
of 8.06 and a maximum of 17.22. The execution-primal adapter does not audit
rate rows, so passing its box-row checks is not a complete QP certificate.

This prevents the tempting workaround of suppressing the execution-primal
check or clamping its output: doing so would hide both the observed box-row
violation and unverified rate-row violations.

## Hypothesis decision

| Hypothesis | Result | Evidence |
|---|---|---|
| H1 mixed-unit global termination admits a locally invalid row | Accepted | physical global tolerance is larger than the rejected row tolerance and the exact returned primal violates that row |
| H2 active-bound formulation amplifies occurrence | Contributor | acceleration/velocity rejects cluster just outside active bounds, but the same global/local contract mismatch exists elsewhere |
| H3 warm transport produces the defect | Rejected | cold and zero-stage warm rejects persist |

## Fix options

1. **Full variable/constraint nondimensionalization of the five-state QP
   (recommended).** Derive characteristic scales from the formulation schema,
   solve in dimensionless coordinates, transform the exact result back to
   physical units, then require every physical row certificate. This repairs
   the producer and permits deletion of the partial/global dual contract.
2. **Enable OSQP scaled termination only.** Smaller implementation, but it
   delegates the safety contract to undocumented internal scaling and still
   does not prove each physical row. Rejected as the root fix.
3. **Tighten tolerances / increase iterations / add cold retry.** These only
   alter frequency or add another authority path. Rejected.
4. **Clamp execution values or loosen row tolerances.** Hides an invalid solve
   and leaves curvature-rate rows unproved. Rejected.

## Required pre-fix test for the repair Slice

Construct a deterministic five-state-shaped mixed-unit QP whose global
progress scale allows OSQP success while an acceleration/velocity/rate row
exceeds its local physical tolerance. The pre-fix solver contract must fail
that test. The repaired formulation must:

- converge without a retry or alternate normal solver;
- return a physical primal satisfying every finite row tolerance;
- preserve the optimum after forward/back coordinate transformation;
- transform primal and dual warm starts under the same schema;
- preserve the existing problem/solution fingerprint and certification join.

## Authority impact

This Slice changes telemetry only. It adds no normal authority, fallback,
feature flag, timeout, tolerance, margin or controller parameter. The current
Emergency behavior therefore remains unchanged pending the root repair.
