# MPCC single-authority migration slices

Each slice is implemented in its own steering. A slice begins with a failing deterministic test or
replay and ends by deleting replaced production branches. Do not combine structural migration with
performance tuning.

## Slice 1: Canonical contracts and fingerprint

### Purpose

Make the current pipeline traceable without changing command behavior.

### Add

- `ControlIntent`: Track, Cruise, Follow, Hold, Stop, ShiftOut, Pass, Return, Rejoin.
- `MpccProblemContext`: observation generation, stage geometry ID, target/obstacle generation,
  horizon, formulation/schema ID, bounds/cost schema ID, and intent generation.
- `CertifiedMpccSolution`: solved status, primal, predicted trajectory, residuals, physical
  certificate, context fingerprint, validity horizon.
- `FinalControlDecision`: normal solution ID or explicit Emergency/Recovery override.

### Delete/replace

- Ad-hoc logging fields that reconstruct solution identity from target/side/age alone.
- Any selection path that cannot name a problem and certificate fingerprint.

### Exit gate

- Every existing control-source transition can be replayed with a complete identity trace.
- No runtime behavior/configuration change.
- Existing test suite and build pass.

## Slice 2: Canonical MPCC for Track/Cruise in shadow

### Purpose

Prove the five-state formulation can represent ordinary racing without involving overtake state.

### Add

- Track and Cruise intent problem construction using the same stage geometry/certificate contract.
- Shadow solve timing, feasibility, prediction and command-difference telemetry.
- Same-formulation warm-start/reset rules.

### Delete/replace

- No production authority deletion yet. Shadow results must not own commands.
- Avoid introducing a second long-lived feature flag; use the existing migration boundary where
  possible and document its deletion gate.

### Exit gate

- Fixed single-car replay and repeated six-lap simulation show no non-finite result, stale adoption,
  physical certificate failure, or 40 Hz deadline overrun attributable to shadow MPCC.
- Track/Cruise MPCC solution coverage meets the threshold defined in `validation-plan.md`.

### 2026-08-22 status

- Exact five-state execution pose and wall-certificate provenance are implemented.
- Mixed-unit solver output now carries per-row residual/tolerance, and Track/Cruise shadow applies a
  metre-domain lateral-row contract before extraction/certification.
- `output/20260822-161428` completed approximately four wraps with 7,662/7,662 solves, 7,627 physical
  certificates (99.54%), one callback overrun, and zero shadow selections.
- Authority promotion remains blocked because 26 hard-contact and 9 swept-path certificate rejects
  remain. Three attempted physical-bound approximations were measured and removed; see
  `.steering/20260822-track-cruise-footprint-bounds/validation.md`.
- Current-pose provenance is now separated from candidate provenance. In
  `output/20260822-164756`, 9,630/9,630 solves produced 9,500 full certificates. The 130 rejects
  split into 54 candidate hard contacts, 73 unsafe legacy-production current poses and 3 genuine
  swept candidate-connection failures. Candidate coverage while the current pose was safe was
  9,500/9,557 (99.40%). See
  `.steering/20260822-mpcc-current-pose-wall-provenance/validation.md`.
- Solved-progress course-frame provenance is now canonical. In `output/20260822-181304`, 4,794/4,794
  solves produced 4,782 certificates (99.75%); candidate discrete hard contacts and course-frame
  provenance failures were both zero. The remaining candidate-side rejects were two genuine
  current-to-first-stage swept failures. See
  `.steering/20260822-track-cruise-wall-bound-contract/validation.md`.
- Slice 3 remains blocked: the first-stage reachability/stitch defect must be resolved, and authority
  handoff from a legacy-created unsafe current pose needs an explicit fail-closed/recovery contract.

## Slice 3: Track/Cruise authority promotion

### Purpose

Make canonical MPCC the sole normal authority for unobstructed racing.

### Fallback rule

```text
fresh certified canonical solution
-> bounded last-certified canonical solution
-> Emergency Stop
```

No cycle-local transfer to three-state or legacy MPC is allowed after promotion.

### Delete/replace

- Track/Cruise legacy-MPC execution branch.
- Formulation handoff smoothing for Track/Cruise.
- Legacy warm-start transition for Track/Cruise.

### Exit gate

- Six-lap repeated runs have zero normal-authority formulation switches.
- Every published non-Recovery command is canonical MPCC or explicit Emergency Stop.
- No regression in wall/contact/finish criteria.

## Slice 4: Follow/Hold/Stop integration

### Purpose

Express longitudinal interaction as stage constraints/references of the same MPCC.

### Add

- Follow distance/relative-speed intent.
- Hold/Stop terminal and progress bounds.
- Dynamic obstacle tube freshness and uncertainty in the problem fingerprint.

### Delete/replace

- Follow-specific normal command owner.
- Low-speed direct pass/rejoin control where it represents normal obstacle handling.
- Speed floors/caps that independently own longitudinal output instead of becoming MPCC bounds.

### Exit gate

- Moving front, stopped front, opening gap, and disappearing target replays all remain in the
  canonical formulation.
- Lateral and longitudinal solution IDs always match.

## Slice 5: Overtake/Dynamic Escape integration

### Purpose

Use Mission/branch/DP outputs as intent and constraints while canonical MPCC owns execution.

### Retain as inputs

- target selection and continuity;
- left/right homotopy evaluation;
- Frenet DP corridor;
- no-return and rear-clear semantics;
- wall/opponent physical bounds.

### Delete/replace

- `convert_extended_solution_to_legacy()` in the live execution path.
- Five-state -> three-state -> legacy cycle fallback.
- DP/Mission paths acting as independent lateral command authorities.
- Dynamic Escape formulation lease whose only purpose is bridging a controller switch.
- wall handoff states made redundant by atomic certified-solution replacement.

### Exit gate

- ShiftOut, Pass, Return, DynamicWait and DynamicEscape all report the canonical formulation.
- No `multiple-lateral-authorities` or split lateral/longitudinal solution identity.
- Candidate loss causes same-formulation continuation, reoptimization, or Emergency Stop—not a
  controller switch.

## Slice 6: Legacy and migration path removal

### Purpose

Finish the architecture simplification instead of leaving permanent dual control.

### Delete

- legacy normal MPC solve path;
- three-state progress fallback path;
- extended reentry/circuit-breaker logic that exists only to arbitrate two formulations;
- formulation handoff smoothing;
- obsolete runtime feature flags, cooldowns and compatibility telemetry;
- direct low-speed normal control authority.

### Retain

- canonical MPCC;
- same-formulation last-certified solution store;
- emergency stop;
- Stuck/gear/reverse Recovery;
- one final command publisher and one final decision trace.

### Exit gate

- Static search finds no normal-driving call to legacy/three-state/direct solve paths.
- Configuration and final control source counts are reduced and documented.
- Full scenario matrix passes.

## Slice 7: Parameter tuning

Only after Slice 6, tune:

- horizon and solve cadence;
- contour/lag/progress/velocity weights;
- acceleration and curvature-rate weights;
- wall and vehicle clearances;
- obstacle uncertainty;
- branch-switch penalties.

Each tuning experiment changes one parameter family against the fixed structural baseline and records
both central performance and failure tails.

## Rollback policy

- Roll back a slice if its invariant fails or its exit KPI regresses; do not restore a removed path by
  adding a new feature flag.
- A temporary migration flag must name its removal slice and may not survive Slice 6.
- The rollback target is the previous accepted slice commit, not an arbitrary older racing build.
