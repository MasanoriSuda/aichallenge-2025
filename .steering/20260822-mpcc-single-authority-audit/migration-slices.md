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
- `.steering/20260822-track-cruise-first-stage-reachability` completed the missing complete-Frenet
  state contract in Track/Cruise shadow. The delayed control pose now supplies initial `e_lag`, solved
  lag is preserved, and lag-aware world reconstruction is used by the shadow wall certificate.
- `output/20260822-194818` classified all three remaining first-stage events as legacy-created
  delay-prefix collisions: raw-to-predicted motion was blocked and the new control rollout began in
  collision at index zero. These were not interpolation-only false positives and were not relaxed.
- Slice 3 is no longer blocked by unknown first-stage provenance. Authority promotion still requires
  an explicit decision because it must begin from fresh canonical certification and may not perform
  a late cycle-local transfer after legacy control has consumed the reachable prefix.

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

### 2026-08-22 preparation status

- `.steering/20260822-track-cruise-canonical-authority-contract` records the current production,
  shadow and fallback flow.
- A pure, runtime-disconnected selector now encodes the only permitted normal chain: fresh certified
  five-state plan, retained certified five-state plan, Emergency Stop.
- The selector requires executable control stages in addition to certificate metadata; an OSQP warm
  start cannot be passed as authority.
- `.steering/20260822-track-cruise-retained-plan-provenance` closes a late-handoff hole in that
  selector. Every candidate now names a real execution plan and a current-decision execution
  certificate. A retained solution keeps its original solver problem identity but cannot own
  control from only its old wall certificate and a claimed remaining-stage count.
- `.steering/20260822-track-cruise-canonical-plan-store` adds the runtime-disconnected complete-plan
  lifecycle needed to back those identities. It atomically stores all five-state predictions and
  three-input stages, rejects stale async replacement, advances an exact non-clamping cursor and
  builds candidates only from current physical revalidation of the actual remaining window.
- `.steering/20260822-track-cruise-canonical-plan-extraction` adds a runtime-disconnected adapter
  that constructs the complete immutable plan directly from the certified five-state primal. It
  preserves lag, acceleration, virtual-progress input and exact stage timing; it does not use the
  lossy legacy conversion.
- `.steering/20260822-track-cruise-canonical-shadow-store` connects that adapter and store to the
  physically certified Track/Cruise shadow producer. Complete plans are now retained with explicit
  extraction/store telemetry while command selection remains `authority=shadow, selected=0`.
  Legacy conversion is comparison-only and no longer gates the canonical plan.
- `.steering/20260822-track-cruise-canonical-fresh-admission` now resolves the exact fresh cursor,
  binds the current physical proof to that plan/window and runs the production canonical selector
  in shadow. A shadow result is certified only if the selector returns `FreshCertified`.
- `.steering/20260822-canonical-actuation-extraction` adds the exact cursor-to-actuation contract.
  It preserves predicted velocity, optimized acceleration, curvature and virtual-progress speed
  without legacy flattening, rejects exhausted/mismatched cursors and verifies the stored result
  against the direct primal in shadow.
- `.steering/20260822-canonical-current-intent-contract` closes the current-supervisor provenance
  gap in the pure selector. Fresh and retained candidates must now match the current Track/Cruise
  intent exactly; a current Follow/Hold/Stop/overtake intent or an old Track/Cruise plan crossing an
  intent transition fails closed before authority promotion.
- Runtime promotion is not yet performed. The remaining implementation must add current-pose
  revalidation for retained plans, obtain dynamic coverage for fresh/retained selection, and then
  connect the same selector to final output while deleting Track/Cruise legacy fallback.
- `.steering/20260822-track-cruise-authority-promotion-gate` freezes the remaining evidence and
  authority decision. Fresh shadow coverage is required first; retained execution then needs
  progress-aligned current wall/obstacle revalidation. Final publisher connection is intentionally
  not performed without those results and explicit approval.
- `.steering/20260822-track-cruise-retained-revalidation-design` audits that retained gap before
  implementation. It prohibits old-certificate and stage-index reuse, separates absolute-progress
  wall sampling from current-relative-time obstacle sampling, and requires measured-to-predicted
  plus predicted-to-horizon swept proofs with current provenance. Fresh Gate A now permits a
  retained shadow connection; production authority remains prohibited until Gate B evidence.
- `.steering/20260822-canonical-numerical-boundary-contract` closes a fresh-path producer/consumer
  mismatch found by Gate A. A solver-certified tiny negative virtual-progress value is now
  normalized only inside its exact box-row tolerance before becoming an executable artifact; the
  raw primal remains untouched. In `output/20260822-232351`, the former 324
  `invalid-control-stage` rejects became zero and all 9,678 physically certified cycles completed
  the fresh canonical chain with zero actuation difference. Three out-of-row-tolerance stage-zero
  values were correctly rejected before canonical certification.
- `.steering/20260822-osqp-rowwise-residual-admission` tested whether the common solver must reject
  every local per-row diagnostic exceedance. `output/20260822-234326` falsified that hypothesis at
  the first legacy MPC curvature-rate row and caused a cold-reset solve-failure cascade. The
  experimental implementation was removed. Gate A treats those three rare cycles as typed fresh
  canonical unavailability for the retained same-formulation path to cover.
- Fresh Gate A is accepted for retained **shadow** implementation: every physically certified
  cycle completed the exact canonical chain with zero actuation difference. Final publisher
  promotion remains an explicit authority boundary.
- This connection is an explicit authority boundary and requires approval before implementation.

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
