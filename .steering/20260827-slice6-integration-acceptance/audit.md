# Slice 6 Audit Record

## Result

The structural integration repairs and static authority audit are complete.
The moving two-vehicle Gate now reaches Track/Cruise, Follow and ShiftOut with
canonical seven-state authority. Dynamic acceptance is still open because the
first observed ShiftOut did not reach Pass/Return: fresh authority was lost
near the wall and the retained continuation was correctly rejected before a
subsequent Emergency/Recovery sequence.

Slice 7 parameter tuning must not start from this record. The earliest
ShiftOut authority/wall-contract failure must be repaired structurally, then
the two- and three-vehicle dynamic Gates must be rerun without changing
controller parameters.

## Failure-first evidence

| Output | Observation | Conclusion |
|---|---|---|
| `20260827-160637` | canonical problem/artifact horizon identities diverged | effective seven-state QP horizon was not the sole identity owner |
| `20260827-163233` | horizon mismatch was removed; first-solve evidence remained absent | horizon repair was necessary but not sufficient |
| `20260827-164623` | first solve had no current-problem warm-start provenance | a current affine bootstrap was required |
| `20260827-165617` | current-problem bootstrap was observed | bootstrap identity and initialization path were connected |
| `20260827-171213` | exact physical replay rejected wall bounds by about 1--2 mm after the configured physical tolerance | refined QP and nonlinear replay represented different vehicle trajectories; this was not a clearance-tuning problem |
| `20260827-172349` | no vehicle motion | invalid dynamic evidence; excluded |
| `20260827-172718` | both vehicles reached `Ready`, but canonical normal authority repeatedly emitted Emergency | controller-side QP/relinearization failure; the earlier startup diagnosis was disproved |
| `20260827-175049` | infinitesimal box residuals no longer invalidated the nonlinear tangent, but the relinearized QP still failed inside a narrow wall trust bucket | projecting the tangent point was necessary but did not repair the pipeline ordering |
| `20260827-175828` | both vehicles moved; Track/Cruise, Follow and ShiftOut used certified seven-state authority | SQP ordering repair is dynamically demonstrated; Pass/Return remains unaccepted |

## Root cause and propagation

The earliest demonstrated controller defect was inconsistent ownership of the
optimization model and its trust region:

```text
semantic reference solve
  -> progress/lag/heading wall buckets are frozen around that provisional path
  -> temporal dynamics are replaced by tangents from a different path
  -> the narrow trust buckets and new affine equalities become incompatible
  -> relinearized QP is rejected or exact nonlinear replay rejects the result
  -> canonical normal authority is unavailable
```

The surface symptoms were a physical-proof rejection and later a QP maximum-
iteration failure. Relaxing wall clearance or adding a fallback would have
hidden the mismatch. The repair performs the single canonical nonlinear
relinearization first, seeds it from the current affine problem, then builds
wall and dynamic-obstacle refinements around that relinearized solution.

The horizon and first-solve defects were upstream contributors: they could
prevent a semantically compatible solve from existing before the physical
proof stage.

## Implemented invariants

- One effective horizon identity is used by solve, bounds, snapshot, retained
  revalidation and artifact certification.
- A bootstrap is derived only from the current QP and never constitutes
  retained execution proof.
- Progress-aligned wall rows and stage-wise dynamic-obstacle rows keep distinct
  ownership and diagnostic names.
- Physical refinements require a successful successive-linearization solve.
- A solver-certified residual may be projected onto exact variable boxes only
  to choose a nonlinear tangent; the solved output itself is never clamped.
- A relinearized QP is bootstrapped from its own affine equality system, never
  from primal/dual provenance belonging to the replaced equalities.
- Exact nonlinear wall/actuator replay remains mandatory and fail-closed.
- Retained execution begins at the current measured state and cannot extend
  past its certified executable prefix.
- DynamicWait cannot erase target/generation or lateral authority.
- A certified candidate becomes retained evidence only after its exact
  serialized actuation is published.
- No legacy three/five-state solver or normal fallthrough is available.

## Verification

- `make autoware-build`: 25 packages built successfully.
- Full `multi_purpose_mpc_ros` test suite: 47/47 CTest targets passed.
- Google/Python test total: 1,977 tests, 0 failures, 0 errors, 0 skipped.
- Single-authority source contract: 63/63 checks passed.
- `git diff --check`: passed.

`colcon test-result --verbose` also reported a stale, unrelated generated build
artifact for `joycon_contract_guard/package.xml`; the package test summary was
still 1,976 tests with no failure. No generated build/output artifact is part
of this change.

## Remaining dynamic Gate

Collect the following without tuning:

1. `make dev2`: Pass/Return after the now-demonstrated Track/Cruise, Follow and
   ShiftOut path.
2. No stale/wrong-generation/unproved artifact publication.
3. Successive-linearization and exact physical replay outcomes for every fresh
   branch used by authority.
4. Retained current-stage and suffix coverage, including DynamicWait.
5. Callback p95/p99/max and consecutive 25 ms overruns.
6. Typed cause before every Emergency or Recovery.
7. Only after the two-vehicle Gate passes, repeat with `make dev3`.

Until these observations exist, Slice 6 is structurally integrated, statically
verified and dynamically proven through ShiftOut, but not accepted for race
quality.
