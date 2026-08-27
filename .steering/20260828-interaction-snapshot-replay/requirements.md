# IM-1 immutable InteractionSnapshot requirements

## Baseline

- Branch: `develop_july`
- Rollback commit: `ce78f928`
- Production controller source remains the `0287d193` frozen baseline.
- Production authority, solver settings, wall clearance and behavior parameters
  must not change.

## Repaired invariant

One rejected canonical seven-state problem must be replayable as one immutable
current-world snapshot.  A/B/C/D consumers must receive the same:

- seven-state initial condition and semantic request;
- exact source problem identity and schemas;
- stage geometry and nominal path distance;
- wall profile, occupancy grid and footprint;
- measured current pose and measured-to-control wall path;
- every current V2X obstacle observation needed to rebuild time-indexed tubes;
- target, generation, selected side and intent;
- exact recorded QP and production outcome.

No consumer may reconstruct a missing field from Mission state, log text,
current runtime state or a different observation epoch.

## Earliest current violation

`mpcc_architecture_snapshot::record_failure` serializes the semantic solver
snapshot and exact QP, but the public loader restores only `RecordedQp`.
The recorded source also lacks a loadable all-vehicle current-world observation
and the exact current-pose/control-prefix inputs used by final physical proof.
Therefore the current artifact is solver-replay-ready but not
architecture-comparison-replay-ready.

## Scope

- Add owned replay-world data to the already immutable async solver snapshot.
- Populate it before background submission from the same current-world epoch.
- Serialize and deserialize the complete comparison input.
- Seal and verify a deterministic interaction fingerprint.
- Preserve exact-QP warm/cold replay compatibility.

## Non-scope

- No InteractionBundle candidate generation.
- No additional solver invocation.
- No left/right/Follow branch selection.
- No command, publisher, Gate-A, retained-plan or Recovery changes.
- No `.steering/ano` MCAP conversion yet; its schema adapter follows after the
  native snapshot contract is deterministic.

## Implementation gate

- Files to change:
  - `include/.../mpcc_rate_resolved_shadow.hpp`
  - `include/.../mpcc_architecture_snapshot.hpp`
  - `src/mpcc_architecture_snapshot.cpp`
  - `src/mpc_controller_cpp.cpp`
  - `test/test_mpcc_architecture_snapshot.cpp`
- Failing test first:
  - complete interaction snapshot round-trip is unavailable before the change;
  - missing/mutated world provenance cannot be accepted as replay-ready.
- Root producer to change: canonical async submission snapshot builder.
- Mask/bypass removed: none in this observation-only Slice.
- New production branches/configuration: zero.
- Remaining legacy authority: unchanged.

## Definition of done

- A complete current-world failure artifact round-trips deterministically.
- Loaded fingerprint equals the recorded fingerprint.
- Changing a wall, vehicle, identity or semantic field invalidates the seal.
- An incomplete old v1 artifact remains loadable for exact-QP replay but is
  explicitly not interaction-replay-ready.
- Focused tests, package tests and `make autoware-build` pass.
- Source audit proves the loaded snapshot has no command-authority API.
