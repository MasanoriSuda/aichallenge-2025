# Audit

## Result

Accepted as a structural root-cause repair. `output/20260827-175828` proves
both vehicles launched and the production publisher used certified canonical
seven-state Track/Cruise, Follow and ShiftOut artifacts.

## Causal chain

```text
old affine solve
  -> narrow wall trust bucket
  -> replace dynamics equalities
  -> stale warm start and incompatible trust/equality system
  -> solve or exact replay rejection
  -> no canonical normal authority
  -> Emergency zero command
```

The repair changes the ownership order, not a numerical threshold.

## Verification

- `make autoware-build`: 25 packages succeeded.
- `ctest --test-dir /aichallenge/workspace/build/multi_purpose_mpc_ros
  --output-on-failure`: 47/47 targets passed.
- Aggregate package tests: 1,977, no failures.
- Dynamic run: `output/20260827-175828`.

## Remaining issue deliberately not patched here

The first ShiftOut later lost fresh and retained authority near a wall and did
not reach Pass/Return. That event is downstream of this repaired solve-start
failure and is being audited separately from its earliest contract mismatch.
