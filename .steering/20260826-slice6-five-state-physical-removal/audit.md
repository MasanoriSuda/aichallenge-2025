# Audit

Baseline: `5a22f95 refactor(mpcc): remove five state overtake gate`.

Static production linking already excludes the old libraries, but that alone
does not meet Slice 6: CMake still builds them and the common formulation enum
still admits their identity.  This Slice treats reconnectability as the defect
and removes the retired implementation rather than adding another availability
flag.

## Causal finding

The old implementation was an isolated but complete authority island.  It had
no production caller after `5a22f95`, yet retained an executable formulation
identity, plan store, current-world revalidation, async transport and CMake
installation path.  A future local fallback could therefore restore the old
owner without an obvious interface error.  The remaining controller helpers
were declaration-only remnants of the same three/five-state execution layout.

The correction deletes this producer vocabulary. It does not translate its
plans into six-state objects and does not leave an availability switch. A
second reachability audit also removed three five-state-only `mpcc_progress`
semantic helpers and `CertifiedWarmStartStore`: each had definitions and
self-tests but no production caller.

## Failure-first evidence

The new physical-deletion source contract failed before implementation on
`canonical_execution_plan.hpp`.  After deletion the complete source suite is:

```text
55 passed
```

Static search finds none of the following in production headers, sources or
CMake:

- `VelocityProgress5State` / `velocity-progress-5state`;
- retired three-state formulation identities and converters;
- canonical five-state plan/retained/Follow async libraries;
- old five/three-state controller wall-proof helpers;
- legacy normal solver/fallthrough telemetry strings.

## Build and tests

- `make autoware-build`: 25 packages succeeded.
- MPCC package: 46/46 CTest targets passed.
- Test summary: 1,887 tests, zero failures/errors/skips.
- `git diff --check`: clean.

Five CTest targets and fourteen self-tests were removed because they tested
only the deleted five-state implementation. The fourteen are exactly twelve
five-state primal/trajectory/constraint tests and two unused warm-start store
tests. Existing six-state, physical proof, retained proof, production adapter,
Overtake and Recovery tests remain. The only
`colcon test-result` diagnostic is the pre-existing stale
`build/joycon_contract_guard/package.xml` lookup.

## Moving acceptance

Run: `output/20260826-163720` (`make dev2`, bounded, then `make down`).

- Both domains moved with repeated six-state production actuation joins.
- Domain 1 admitted `Idle -> ShiftOut` using
  `gate=six-state-shiftout`, `certificate=1`, `samples=20`,
  `exact_stages=20`.
- Five-state formulation/decision traces: 0.
- Domain 2 callback overrun windows: 0.
- Domain 1 later left ShiftOut because `locked target stale or lost`.
- Both domains still showed short explicit Emergency episodes caused by
  `rate-resolved authority unavailable/retained-proof-unavailable`.

The latter two findings also exist in the pre-deletion run
`output/20260826-161516`; they are not a physical-deletion regression.  They
remain the first post-Slice-6 integration-quality investigation.  Restoring a
five-state fallback or tuning solver/wall parameters is not an accepted fix.

## Acceptance

Accepted as the final structural Slice 6 deletion.  Normal formulation
authority is physically single-owner.  Emergency and Recovery remain the only
intentional external overrides.  Race-quality acceptance is not claimed by
this structural result.
