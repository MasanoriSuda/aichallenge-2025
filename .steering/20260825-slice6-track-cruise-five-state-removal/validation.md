# Validation

## Evidence boundary

- Branch: `develop_july`
- Baseline: `9a4da6a refactor(mpcc): promote six-state track cruise authority`
- Dynamic baseline: `output/20260825-100454`
- This Slice deletes an unreachable owner; it does not alter runtime policy,
  parameters, ROS interfaces or the accepted six-state Track/Cruise owner.
- User-owned `aichallenge/result-summary.json` was neither edited nor staged by
  this Slice.

## Root cause and failure-first proof

The six-state authority migration made the five-state Track/Cruise owner
unreachable, but the shared Track/Cruise/Rejoin evaluator, retained store,
solver context and telemetry remained compiled. That dead graph obscured the
real production owner and could be reconnected by a later local patch.

A source-contract assertion was added before the deletion. It failed with one
failure and 33 passes because `CanonicalNormalShadowMode` and the retired
Track/Cruise objects still existed. After the deletion the focused contract
suite reports 34 passes.

## Structural change

- Removed the Track/Cruise five-state retained evaluator, plan store, solver
  context, warm-start identity, context epoch and telemetry.
- Removed the Track/Cruise/Rejoin mode enum and converted the shared evaluator
  into the explicit `evaluate_rejoin_canonical()` responsibility.
- Kept Rejoin's five-state canonical production behavior, its own store,
  solver context and telemetry.
- Kept the live rate-resolved six-state Track/Cruise worker, proof store and
  production adapter unchanged.
- Added no compatibility flag, alternate fallback, timeout, clamp or parameter.

## Static validation

- Exact-symbol search: no retired five-state Track/Cruise store, solver,
  retained evaluator, mode or telemetry remains in production source.
- `test_single_authority_source_contract.py`: 34 passed.
- `make autoware-build`: 25 packages passed.
- Package CTest after rebuilding with `BUILD_TESTING=ON`: 49 targets passed.
- `colcon test-result --verbose`: 1,869 tests, zero errors, failures or skips.

An initial package-test attempt used stale pre-Slice test executables after the
ordinary production build and produced two stack-smash failures. Rebuilding
the package test targets with `BUILD_TESTING=ON` removed the ABI mismatch; the
complete rebuilt suite then passed. This was a validation-artifact issue, not
a runtime fallback or a reason to change controller behavior.

## Dynamic evidence

No new dynamic trial is required for this bounded deletion because the
production Track/Cruise call graph was already six-state-only at the baseline.
`output/20260825-100454` remains the accepted dynamic evidence: exact six-state
identity reached publication with zero command mutation and no cross-
formulation normal fallback.

## Residual scope

- Rejoin, Follow and Overtake still use five-state canonical responsibilities;
  they require separate authority/deletion Slices.
- Historical `shadow` names on the live rate-resolved Track/Cruise transport
  remain. They are naming debt, not an alternate owner, and should be cleaned
  mechanically after authority deletion so search results become unambiguous.
- Slice 6 is not complete until all remaining legacy/three-state normal paths
  and migration-only arbitration are physically removed.
