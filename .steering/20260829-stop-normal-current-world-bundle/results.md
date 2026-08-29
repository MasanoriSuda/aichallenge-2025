# Results: Stop-to-normal current-world bundle

## Static verification

- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q .../test_single_authority_source_contract.py`
  - 75 passed.
- Focused CTest
  - retained revalidation and production adapter: 2/2 passed.
- Full package CTest
  - 54/54 targets passed.
- `make autoware-build`
  - 25 packages completed successfully.

## Dynamic verification

- Baseline: `output/20260829-214906`, commit `eb3b4fc2`.
- Candidate: `output/20260829-220933`, `make dev2`.

D1 emitted 121 canonical decisions whose solver source was
`current-world-bundle`.  Of the decision transitions visible in the aggregated
trace, 114 immediately followed an Emergency decision.  The vehicle recovered
from near zero speed and later reached 7.95 m/s; the baseline ended at
0.0 m/s and never formed Bundle authority.

The common wall/dynamic/Follow/terminal proof remained mandatory.  Source-plan
execution promotion rejects were zero.  No wall, clearance, timing, solver or
behavior configuration changed.

## Acceptance

Accepted for the narrow Stop-to-normal publication invariant.  Not accepted
as an Overtake or full six-lap quality Gate: later Recovery, short Emergency
insertions and callback overruns remain and must be investigated from their
own first violated invariant.
