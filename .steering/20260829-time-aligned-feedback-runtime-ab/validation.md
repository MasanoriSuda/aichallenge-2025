# Validation

## Static verification

- `make autoware-build`: 25 packages passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 54/54 targets,
  2,099 tests, zero errors, failures or skips.
- `test_single_authority_source_contract.py`: 75/75 passed.
- Focused common-clock suffix regression: 4/4 passed.

## Offline replay

- Frozen mixed-origin final-QP reuse remained infeasible.
- A current `SolverContext` preparation rebuilt as one time-aligned suffix
  accepted the unchanged exact physical adapter.
- Representative total suffix costs: 8.294 ms and 22.404 ms.
- Corresponding full semantic rebuilds: approximately 46.80 ms and 99.96 ms.

## Dynamic observation-only A/B

Run: `output/20260829-182105` (`make dev2`).

| Metric | D1 | D2 |
|---|---:|---:|
| completed feedback results | 1,469 | 165 |
| feedback QP accepted | 671 | 153 |
| exact physical accepted | 671 | 153 |
| current dynamic world clear | 241 | 51 |
| retained current-world authority-ready | 122 | 35 |
| mean compute | 24.22 ms | 27.42 ms |
| maximum compute | 108.11 ms | 77.77 ms |

Production Store and command authority were unchanged.  The temporary live
observation worker was removed after this run because its unbounded trigger
rate was itself an unacceptable scheduling load.

## Classification

The result matches the architecture audit classification
`offline succeeds and live direct adoption fails`: scheduling/lifecycle
defect.  It does not justify a clearance, solver tolerance, timeout, lease or
fallback change.

The next Slice may design one bounded production connector, but it must remove
direct stale adoption for the same source result atomically and retain exactly
one certified Store/publisher owner.
