# Live validation

## Run

- logs: `output/20260823-171813/d1/autoware.log`,
  `output/20260823-171813/d2/autoware.log`
- isolated simulator result: `output/20260823-follow-live-positive-gate/`
- production parameters and command authority: unchanged
- user result SHA-256 before/after:
  `03e2f3935d95a550d0e1a3f2006dde08dae4a7a4c74121c430b2452daf4414e6`

Domain 1 produced 29 complete Follow telemetry windows before the evidence threshold was reached.

## Aggregate Follow result

| Boundary | Count | Rate versus valid attempt |
|---|---:|---:|
| eligible / typed contract / build / attempts | 835 | 100.00% |
| QP solved and normalized | 753 | 90.18% |
| actuation extracted | 753 | 90.18% |
| effective gap and physical wall certified | 753 | 90.18% |
| canonical plan/cursor/candidate/authority/actuation/command | 753 at every boundary | 90.18% |
| canonical-ready shadow | 753 | 90.18% |
| warm starts applied | 746 | 89.34% |
| solver context resets | 5 | 0.60% |

All 753 solved/normalized results passed every downstream physical and canonical boundary. All Follow
event logs remained `authority=shadow, selected=0`.

Compared with the pre-fix live baseline:

| Metric | Baseline | Current | Change |
|---|---:|---:|---:|
| valid-attempt canonical-ready rate | 82.73% | 90.18% | +7.45 percentage points |
| warm starts applied | 0 | 746 | structural defect removed |

## Runtime

- build: 0.048 ms average, 0.919 ms maximum
- solve: 3.128 ms average, 45.438 ms maximum
- certificate/canonical chain: 1.642 ms average, 5.057 ms maximum
- total Follow shadow: 4.696 ms average, 45.502 ms maximum

Average total time improved from the baseline 6.111 ms. The maximum remains above the 25 ms control
period and therefore fresh-only production authority is not acceptable.

## Remaining first failing boundary

The 82 unavailable cycles were all solve-boundary losses:

- 8 physical-row post-solve rejects, all classified as `other`; transition provenance identifies the
  affected row as the acceleration input box;
- 74 solver-status failures, mainly cold convergence around context/encounter entry;
- zero Follow effective-gap row rejects;
- zero physical wall rejects after normalized solve;
- zero canonical-chain rejects after physical certification.

Once a warm solution was established, multiple consecutive windows reached 40/40 or 41/41 accepted.
The remaining risk is therefore availability during cold/context transitions, not the Follow gap,
wall certificate, or canonical conversion.

## Race context

The bounded run completed one 54.01-second lap and recorded one crash and one wall penalty under the
unchanged production controller. Follow canonical output was shadow-only, so these events neither prove
nor disprove its command quality. They remain production-behavior debt outside this narrow gate.

## Gate decision

The warm-start integration and fresh Follow canonical chain pass their positive live evidence gate.
Production promotion remains **blocked** because a fresh-only selector would convert the 82 unavailable
cycles into Emergency Stop. The next structural slice must provide a bounded, current-world-revalidated
same-formulation retained Follow candidate before any authority connection or scalar-owner deletion.

No solver tuning, margin adjustment, authority promotion or legacy deletion is justified by this slice.
