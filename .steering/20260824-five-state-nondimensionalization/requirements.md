# Requirements

## Evidence boundary

- Baseline: `a56c19e`
- Root-cause evidence: `output/20260824-022828`
- Upstream audit: `.steering/20260824-osqp-convergence-provenance`
- Preserve user-owned `aichallenge/result-summary.json`.

## Repaired invariant

A canonical five-state result may be exposed to any Track, Cruise, Follow or
Overtake consumer only when the exact returned physical primal satisfies every
finite QP row under the same per-row physical tolerance contract.

## Scope

- Define one explicit variable-coordinate scaling contract for the five-state
  formulation.
- Transform `P`, `q`, `A`, primal warm start and returned primal consistently.
- Preserve row scaling and physical dual-coordinate conversion.
- Make every canonical five-state solver context use that same numerical
  contract.
- Require every physical QP row, including dynamics and curvature-rate rows,
  before returning a successful result.
- Add failure-first and coordinate round-trip tests.

## Non-scope

- No vehicle-facing tolerance, iteration, internal Ruiz-scaling or polish
  tuning. The row-normalized policy must disable OSQP's second/global relative
  term after the physical relative tolerance has been embedded per row.
- No vehicle, wall, clearance, speed, acceleration or cost parameter tuning.
- No retry, fallback, timeout, lease, feature flag or alternate normal solver.
- No legacy three-state solver behavior change beyond preserving its existing
  interface until deletion.

## Deletion gate

- Remove the five-state `None` / global-tolerance admission path.
- Remove the partial-certification assumption that execution-box checks alone
  can make a globally admitted five-state result executable.
- Legacy three-state/global admission remains only as an explicitly recorded
  migration authority and is not expanded.

## Acceptance

- Deterministic mixed-unit and asymmetric-bound contracts fail before the fix
  and pass after nondimensionalization.
- Forward/back coordinate transforms preserve optimum and warm-start meaning.
- Focused tests, full package tests and `make autoware-build` pass.
- A bounded `dev2` run has zero five-state execution-primal reject caused by a
  row accepted only through the mixed-unit global tolerance, and every
  certified result has maximum normalized physical-row violation at most 1.
