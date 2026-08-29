# Results: Follow authority semantic alignment

## Static verification

- `make autoware-build`: passed, 25 packages.
- Correct workspace overlay package test:
  - `/aichallenge/workspace/build/multi_purpose_mpc_ros`
  - 54 CTest targets passed.
  - 2091 tests, 0 failures.

An earlier diagnostic invocation used `/aichallenge/build`; it was discarded
and is not validation evidence.

## Dynamic verification

Run: `output/20260829-145849` (`make dev2`).

| Evidence | D1 | D2 |
|---|---:|---:|
| `action=follow/longitudinal_owner=racing-line` | 0 | 0 |
| certified `action=follow/longitudinal_owner=follow-cap` samples | 4 | 0 |
| `action=cruise/longitudinal_owner=racing-line` samples | 31 | 9 |

The removed behavior-label-only authority did not recur. A tactical Follow
label without a real longitudinal cap remained Cruise and retained the normal
dynamic-obstacle contract.

## Newly isolated blocker

This Slice did not make the run acceptable. D1 still entered Emergency and
Stuck Recovery. The first race-time authority loss was now attributable to a
real Cruise-to-Follow-cap transition:

```text
decision 1072:
  requested action=follow, owner=follow-cap
  previous certified Cruise artifact remained published

decision 1123:
  proposed Follow current-world authority unavailable
  previous Cruise current-world proof=progress-lift-rejected
  Gate A proposal unavailable
  canonical Follow Emergency Stop
```

Frozen Follow failures at sequences 531 and 586 classify identically:

- persistent A fails;
- stateless left B fails;
- stateless right B passes the unchanged seven-state SQP, exact wall proof and
  exact dynamic proof;
- terminal progress is 16.35 m / 15.61 m respectively.

Therefore the remaining transition failure is not physical infeasibility and
not justification for a solver, timeout or clearance adjustment. It exposes a
separate architecture question: whether a scalar Follow cap should change the
canonical intent/formulation at all, or instead remain a longitudinal
constraint inside continuous opponent-aware Cruise MPCC.

That question is intentionally deferred to a new architecture Slice. This
Slice closes only the proven semantic mismatch it set out to remove.
