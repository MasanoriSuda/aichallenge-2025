# Slice 6 Audit Record

## Result

The structural integration repairs and static authority audit are complete.
Dynamic acceptance is not closed: the clean two-vehicle run reached AWSIM
`Ready`, but AWSIM did not start vehicle motion. This is the previously
deferred `make dev` / `make dev2` startup integration issue, not evidence that
the MPCC race Gate passed or failed.

Slice 7 parameter tuning must not start from this record. First restore the
documented development startup path, then rerun the two- and three-vehicle
dynamic Gates without changing controller parameters.

## Failure-first evidence

| Output | Observation | Conclusion |
|---|---|---|
| `20260827-160637` | canonical problem/artifact horizon identities diverged | effective seven-state QP horizon was not the sole identity owner |
| `20260827-163233` | horizon mismatch was removed; first-solve evidence remained absent | horizon repair was necessary but not sufficient |
| `20260827-164623` | first solve had no current-problem warm-start provenance | a current affine bootstrap was required |
| `20260827-165617` | current-problem bootstrap was observed | bootstrap identity and initialization path were connected |
| `20260827-171213` | exact physical replay rejected wall bounds by about 1--2 mm after the configured physical tolerance | refined QP and nonlinear replay represented different vehicle trajectories; this was not a clearance-tuning problem |
| `20260827-172349` | no vehicle motion | invalid dynamic evidence; excluded |
| `20260827-172718` | both vehicles reached `Ready`; control-mode publication had a subscriber, but motion never started | dynamic Gate blocked by development startup integration; excluded from MPCC acceptance |

## Root cause and propagation

The earliest demonstrated controller defect was inconsistent ownership of the
optimization model:

```text
semantic reference linearization
  -> physical wall/dynamic rows modify the solved path
  -> temporal dynamics remain linearized around the old path
  -> exact nonlinear replay sees a different trajectory
  -> certified artifact is rejected
  -> canonical normal authority is unavailable
```

The surface symptom was a physical-proof rejection. Relaxing wall clearance or
adding a fallback would have hidden the mismatch. The repair instead makes the
final physical rows and temporal dynamics describe the same iterate before an
artifact may be certified.

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
- Google/Python test total: 1,976 tests, 0 failures, 0 errors, 0 skipped.
- Single-authority source contract: 63/63 checks passed.
- `git diff --check`: passed.

`colcon test-result --verbose` also reported a stale, unrelated generated build
artifact for `joycon_contract_guard/package.xml`; the package test summary was
still 1,976 tests with no failure. No generated build/output artifact is part
of this change.

## Remaining dynamic Gate

After startup integration is restored, collect the following without tuning:

1. `make dev2`: Track/Cruise, Follow and at least ShiftOut; Pass/Return when the
   physical scene permits.
2. No stale/wrong-generation/unproved artifact publication.
3. Successive-linearization and exact physical replay outcomes for every fresh
   branch used by authority.
4. Retained current-stage and suffix coverage, including DynamicWait.
5. Callback p95/p99/max and consecutive 25 ms overruns.
6. Typed cause before every Emergency or Recovery.
7. Only after the two-vehicle Gate passes, repeat with `make dev3`.

Until these observations exist, Slice 6 is structurally integrated and
statically verified, but not dynamically accepted for race quality.
