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
- `.steering/20260822-track-cruise-retained-revalidation` now has dynamic Gate B shadow evidence.
  In `output/20260823-014243`, a typed fresh numerical rejection at decision `20804` was replaced
  in shadow by a prior canonical plan revalidated against the current wall, control prefix and an
  explicitly observed empty V2X world.  Candidate selection and actuation extraction completed,
  while a separate wall-contact interval was rejected as `delay-prefix-blocked`.  Both paths
  remained `selected=0`; production promotion still requires explicit approval.
- This connection is an explicit authority boundary and requires approval before implementation.

### Delete/replace

- Track/Cruise legacy-MPC execution branch.
- Formulation handoff smoothing for Track/Cruise.
- Legacy warm-start transition for Track/Cruise.

### Exit gate

- Six-lap repeated runs have zero normal-authority formulation switches.
- Every published non-Recovery command is canonical MPCC or explicit Emergency Stop.
- No regression in wall/contact/finish criteria.

### 2026-08-23 closure status

- `output/20260823-121707` completed an uninterrupted six-lap production run at
  `46.456 / 43.560 / 43.515 / 44.400 / 43.285 / 42.885 s` with zero callback
  overrun, wall/contact event, abrupt speed loss, confirmed Stuck, Reverse or
  Recovery.
- All 11,002 executable fresh candidates were physically certified and carried
  exact canonical actuation to publication.  No Track/Cruise legacy/three-state
  normal authority was published.
- 676/11,678 eligible cycles had no executable fresh candidate: 50 solve
  unavailability and 626 strict semantic-boundary rejects.  These remained
  explicit canonical Emergency outcomes and did not become a formulation
  switch.
- Seven distinct numerical/formulation corrections were dynamically falsified
  and removed.  The remaining mixed-unit OSQP availability defect is recorded
  as a visible future solver-quality risk rather than hidden by tuning,
  downstream repair or a compatibility fallback.
- Slice 3 architecture migration is accepted.  Slice 4 may begin in
  audit/shadow mode; Follow/Hold/Stop production promotion remains a separate
  approval boundary.  See `.steering/20260823-track-cruise-slice3-closure/`.

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

### 2026-08-23 Follow shadow status

- `.steering/20260823-follow-longitudinal-contract` implements a pure typed
  stage-wise Follow contract and a dedicated five-state shadow solve.
- The first dynamic run exposed that a retained `Follow` action can exist with
  no current front observation (`front=0`, `fd/fs=inf`). Shadow admission now
  rejects that semantic mismatch before contract construction instead of
  misclassifying it as invalid configuration.
- Moving, stopped, opening-gap and invalid-observation behavior is covered by
  deterministic tests. The available `dev2` run produced stopped-front
  `LowSpeedAvoidance`, not moving Follow, so positive moving-front dynamic
  coverage is still required.
- Follow remains shadow-only. Its scalar production owner is intentionally not
  deleted until the positive dynamic gate and a separate authority approval.
- `.steering/20260823-follow-retained-current-world` closes the static retained-lifecycle gap.
  A fresh Follow plan is now stored only after its complete canonical command chain succeeds.
  A later fresh miss can be checked against the current coherent target tube, physical
  `theta + e_lag` gap, current wall, exact retained cursor and the same canonical selector.
- The retained path is shadow-only and has deterministic fail-closed coverage for target/tube
  mutation, malformed or short horizon, current/stage gap violations and wall/provenance failures.
  Static validation passes 38/38 package tests. In `output/20260823-181103`, the first dynamic
  retained window re-certified 5 of 8 fresh-miss attempts through world proof, candidate, selector,
  exact actuation and command reconstruction; a later window added one more retained command.
  All stayed `authority=shadow, selected=0`.
- This is enough to accept the retained mechanism, but not production promotion. The same run has
  intervals where the retained certificate expires while fresh Follow remains unavailable, and it
  records callback overruns. A separate coverage/latency authority gate remains mandatory.
- `.steering/20260823-follow-async-canonical-producer` removes the fresh Follow solve from the 40 Hz
  callback and gives one latest-only worker ownership of the sealed immutable Follow problem.
  Deterministic replay `output/20260823-200200-replay` produced 1404 accepted worker results,
  574 live current-world-ready plans, zero worker exceptions and zero observed callback overruns.
  Snapshot-context validation also rejects re-derived Cruise authority and mismatched fingerprints.
- The Follow asynchronous producer and current-world proof are therefore accepted for promotion
  input. Production remains shadow-only: the next explicit Slice must connect the same canonical
  selector to final output and delete the Follow-specific normal command owner atomically. It may
  not retain a cycle-local scalar/legacy fallback or begin parameter tuning.
- `.steering/20260823-follow-canonical-authority-promotion` completes that authority promotion.
  One typed selection now preserves the exact command, problem, certified solution, immutable plan,
  cursor and prediction through the final publication adapter. `ControlIntent::Follow` returns at
  this boundary: a complete current-world-certified selection publishes canonical normal authority;
  missing or unsafe evidence emits canonical emergency authority and cannot fall through to another
  normal formulation.
- Deterministic production replay
  `output/20260823-202408-follow-production-replay/d1/autoware.log` observed matching
  `intent=follow`, five-state problem/solution/plan/decision provenance and
  `canonical-follow-retained-published`. The final steady Follow window accepted 39/39 selections;
  its worker completed 244/244 submitted jobs with zero exceptions or identity rejects. The only two
  replay traces named `legacy-mpc-solved` were Stop intent, not Follow. Follow is therefore promoted
  and its legacy normal owner is deleted. Hold/Stop remain separately incomplete below.

### 2026-08-23 Hold/Stop intent provenance status

- `.steering/20260823-hold-stop-intent-provenance` found that the existing
  `DynamicWait` action combines rolling replan prefixes and held lateral
  Mission paths; neither is evidence of a longitudinal zero-progress Hold.
- One pure canonical-intent resolver now preserves the committed ShiftOut/Pass
  origin for both forms, maps SafetyBrake to Stop and fails closed on
  incomplete combinations.
- Both the MPCC problem fingerprint and final published-command trace consume
  that resolver. `output/20260823-132619` verified Track, Cruise and Follow
  telemetry joins; the short run did not exercise DynamicWait.
- No Hold/Stop QP, command selection or production authority was added. A Hold
  shadow is blocked until a real longitudinal-hold producer is identified;
  DynamicWait must remain ShiftOut/Pass.
- `.steering/20260823-stop-emergency-authority-boundary` separates the proven Stop case from that
  blocked nominal Hold work. The only observed Stop producer is SafetyBrake, an explicit emergency
  supervisor action. Previously it resolved `intent=stop` but fell through to a legacy normal solve
  before downstream braking, creating split authority.
- Stop now returns before low-speed direct and every normal solver through canonical emergency
  authority. A separate supervisor-intent field preserves `intent=stop` without inventing solver
  identity. Accepted replay `output/20260823-214300-stop-authority-replay-v2/d1/autoware.log`
  observed 10/10 Stop traces as `emergency-override`, `formulation=unresolved`,
  `canonical=satisfied`, and zero legacy Stop traces. Stop emergency integration is complete;
  nominal Hold remains blocked for the producer reason above.
- `.steering/20260823-low-speed-direct-authority-retirement` retires the obsolete Gate2
  stopped-vehicle direct normal owner. The stopped-vehicle gap/local path remains an MPCC planning
  input, but no call site can activate or publish `LowSpeedDirect`. Deterministic replay changed
  `formulation=low-speed-direct` from 31 to zero and `prediction-unavailable` from 44 to zero; the
  first reproduced stopped-front decision at 6.30 m published a prediction-backed five-state MPCC
  result. This closes the low-speed direct part of Slice 4. The replay also exposes the existing
  five-state-to-three-state normal fallback, which remains Slice 5 work rather than a reason to
  restore the direct controller.

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

### 2026-08-24 runtime replacement artifact status

- `.steering/20260824-overtake-runtime-replacement-canonical-artifact` removes the Mission-only
  same-side/cross-side replacement boundary. The selected tactical result is now one typed Mission
  plus immutable five-state canonical plan; the consumer stores that exact plan before exposing a
  new Mission generation, side or phase.
- The asynchronous producer now seals prospective authority identity from the immutable live
  request. Worker-private tactical FSM mutation can no longer increment generation or change
  phase/side semantics of an executable result.
- `output/20260824-114633` accepted `Idle -> ShiftOut` at generation 1 and published the first
  sampled ShiftOut command as `canonical-shiftout-retained`, with no Overtake async-pending or
  legacy-normal handoff. No complete runtime replacement became available in that run.
- Slice 5 remains open: DynamicWait had no current-side canonical prefix and a later retained plan
  failed current-origin/corridor proof. These are follow-up current-world/cursor defects, not a
  reason to restore Mission-only authority or tune clearances.

### 2026-08-23 fresh-chain shadow status

- `.steering/20260823-overtake-canonical-fresh-shadow` connects the already solved live five-state
  result to exact primal normalization, actuation/trajectory extraction, swept wall certification
  and the canonical plan adapter in telemetry-only shadow mode. It does not add another solve or
  change final authority.
- Bounded replay evaluated 405 cycles. 352 exact Overtake artifacts passed normalization,
  actuation/trajectory extraction and the swept physical certificate; canonical/direct first
  actuation difference was zero and shadow evaluation stayed below 1.104 ms in the observed run.
- All 352 were then rejected before canonical plan construction because the shared normal-intent
  contract permits only Track/Cruise/Follow. `ShiftOut`, `Pass` and `Return` are therefore
  structurally unable to become canonical even when their exact artifact is physically certified.
- The next bounded Slice is the Overtake canonical-intent contract. It must extend exact-intent
  support and rerun this shadow without promoting authority. Solver, clearance and weight tuning
  remain prohibited until that contract passes.
- `.steering/20260823-overtake-canonical-intent-contract` closes that intent-domain defect. The
  canonical normal contract now admits ShiftOut/Pass/Return, requires target identity for every
  Follow/Overtake intent and retains exact-intent matching. Replay advanced 353/353 physically
  certified artifacts through plan/cursor/candidate/authority/command and world prediction with
  zero actuation difference; `unsupported-intent` became zero.
- The next fresh Gate A blocker is 37 stage-zero lateral-row violations (roughly 0.057--0.097 m
  against 0.0164 m tolerance). Audit x0 and stage-zero bound provenance before authority promotion;
  do not tune tolerance or wall clearance.
- `.steering/20260823-progress-aligned-wall-contract` replaces fixed-spatial wall boxes in the
  five-state problem with progress-coupled piecewise-affine wall rows. Its accepted replay advanced
  794/794 eligible fresh chains through physical certification with zero wall reject; no wall margin
  or solver tolerance was relaxed.
- `.steering/20260824-overtake-retained-current-world` adds the missing same-formulation retained
  evidence in shadow. A complete fresh Overtake plan is stored immutably and can continue only after
  exact cursor, current target/corridor fingerprint, control pose, course frame, progress branch,
  wall path, and every remaining dynamic-corridor segment are re-certified.
- Two final typed-outcome replays covered 85.5--85.6% of eligible Overtake cycles with fresh or
  retained canonical authority. The retained path completed 58--59 cycles per run, but 14.4--14.5%
  remained fail-closed, led by certificate expiry, progress discontinuity, and insufficient current
  course-frame coverage. Retained proof cost was 0.162 ms average / 0.760 ms maximum in the final
  replay. The mechanism is accepted as shadow infrastructure; production promotion and legacy
  deletion remain blocked until the upstream continuity contract yields complete coverage.
- `.steering/20260824-overtake-retained-live-gate` attempted the required closed-loop `dev2` gate at
  `output/20260824-005436`, but an earlier Track/Cruise canonical failure made the Overtake result
  inadmissible. Domain 2 rejected 153 solved five-state primals at exact acceleration, velocity, or
  virtual-progress bounds and published emergency authority 184 times. The earliest rejection was
  already present before the race session. The dedicated Track/Cruise solver context alone was
  initialized without the canonical row-normalized physical-unit policy used by Follow and live
  extended Overtake. Fix and dynamically prove that initialization contract before repeating the
  Overtake live gate; no tolerance/config/fallback change is authorized.
- `.steering/20260824-track-cruise-row-tolerance-contract` tested that initialization hypothesis
  and rejected it. In `output/20260824-011002`, Domain 1 alternated 33 certified cycles with 32
  solve failures and Domain 2 alternated 10 with 9. Strict row-normalized admission exposed
  input-bound rows 210/212/270 which do not reliably converge inside their own physical
  tolerance; it did not provide stable authority. The source experiment was removed. Audit the
  five-state input-row construction and warm-start transport next, before changing solver
  tolerances or expanding retained-plan eligibility.
- `.steering/20260824-mpcc-warm-start-dual-semantics` tested whether the two category-changing
  dual boundaries caused the warm-only rejects. The source experiment passed 1700 tests and the
  25-package build, but `output/20260824-013035` still produced eight D2 execution-primal rejects,
  all with `warm=1`. The dual patch was removed. Inspection then found the more upstream lifecycle
  break: `solve_extended_progress_problem()` publishes every OSQP-successful result into warm-start
  history before semantic execution-primal normalization and physical certification. A downstream
  rejected result can therefore seed the next horizon. Correct that publication boundary before
  retrying strict Track/Cruise row normalization; do not add a cold-retry flag or relax bounds.
- `.steering/20260824-certified-warm-start-publication` corrects that lifecycle break. Time,
  progress origin, normalized primal and dual now form one single-use certified artifact. A solve
  consumes the prior artifact and only Track/Cruise, Follow, Overtake or branch acceptance may
  publish its replacement. The 1702-test suite and 25-package build pass. In
  `output/20260824-014849`, the previous persistent warm reject chain became a deterministic
  `cold-certified -> warm-rejected -> cold-certified` sequence (D2: 165 certified / 168 rejected),
  proving rejected evidence no longer survives. The remaining defect is the one-stage warm
  transform itself: 151/168 warm rejects were stage-zero acceleration. Make transport elapsed-time
  aware next; do not restore pre-certification storage or add cycle-local cold retry.
- `.steering/20260824-stage-aligned-warm-start-transport` tested the more precise spatial-grid
  form of that hypothesis. The identity resolver's exact/rolling overlap offset was connected to
  primal and dual transport, built, and passed 1679 tests. In bounded runtime
  `output/20260824-020904`, 189 warm outcomes used the intended zero-stage alignment, but Domain 2
  still produced 212 execution-primal rejects (195 warm, 17 cold). The experiment was therefore
  rejected and all source/test changes were removed. The next earliest break is the mismatch
  between OSQP's successful convergence report and the downstream per-physical-row certificate;
  inspect scaled/unscaled residual provenance and solver settings before modifying tolerances.
- `.steering/20260824-osqp-convergence-provenance` added observation-only provenance and accepted
  that mismatch as the next root cause. In bounded pre-race `dev2` evidence
  `output/20260824-022828`, Domain 2 produced 31 certified and 34
  execution-primal-rejected Track/Cruise outcomes even though every solve reported OSQP success.
  The common unscaled/global physical tolerance was approximately 0.015--0.019 because progress
  rows carried 14--18 m scales, while rejected acceleration, predicted-velocity and virtual-progress
  rows had their own approximately 0.001--0.005 tolerances. Moreover many currently certified
  outcomes exceeded local curvature-rate row tolerances, with maximum normalized violation 17.22.
  Active command bounds amplify the failure frequency, but warm transport is falsified as the root.
  Repair the five-state variable/constraint nondimensionalization and require a complete physical-row
  certificate next; do not tune OSQP, clamp commands, loosen bounds or add cold retry.
- `.steering/20260824-five-state-nondimensionalization` repairs that common numerical contract.
  Variable-coordinate and row-tolerance scaling now preserve one exact physical certificate;
  `output/20260824-031300` observed zero `stage=constraint_check` failures in Track/Cruise and
  Follow. No vehicle parameter, clearance, retry or fallback was added.
- `.steering/20260824-overtake-live-gate-after-numerical-repair` then repeated the live Gate at
  `output/20260824-031752`. Two Overtake episodes proved that the numerical blocker is gone but
  exposed the next structural defect. Of 322 evaluated cycles, 198 completed the exact fresh
  canonical chain; all 58 retained attempts failed (40 expired cursors, 18 unavailable course-frame
  windows), while production used 102 circuit/reentry/three-state fallback cycles. Final joined
  traces contained no canonical-satisfied Overtake state: certified five-state emissions lacked the
  canonical command identity and the rest were legacy bypass or wall hold. Production promotion is
  therefore still blocked. Repair the Overtake canonical producer lifecycle and same-formulation
  continuity before connecting the publisher; do not patch only the command identity.
- `.steering/20260824-overtake-canonical-async-producer` adds that independent lifecycle in
  shadow. Exact intent/target/generation/fingerprint jobs now run on a dedicated latest-only
  five-state worker and cross the mailbox only as complete immutable plans; the live controller
  then repeats target/corridor/wall/control-pose proof before selection. Three failure-first gates
  removed clone-local intent/context re-derivation and the stale Track/Cruise/Follow-only
  warm-start intent list. In `output/20260824-043223`, ShiftOut and Pass produced 155 complete
  physical chains and 123 stored current-world plans with zero worker exception, identity reject,
  snapshot failure or callback overrun. Production promotion remains blocked: 34/158 eligible
  cycles lacked current-world selection, led by 23 course-frame-window and 9 current-corridor
  rejects. Audit and close that proof coverage before atomically connecting canonical Overtake and
  deleting conversion/circuit/reentry/three-state normal ownership.
- `.steering/20260824-retained-course-frame-window` repairs the first proof-coverage root cause
  without relaxing any proof. Retained continuity permitted the newly measured progress to be
  slightly ahead of the retained expected-current state, but current course-frame provenance began
  at measured progress and extended only forward. Follow and Overtake therefore failed geometry
  reconstruction after already accepting progress continuity. The shared current course-frame
  window now covers the closed interval from the earliest measured/retained state through the
  retained horizon. The 1723-test suite and 25-package build pass. In
  `output/20260824-045351`, 396 eligible Overtake shadow cycles produced zero course-frame rejects;
  298 completed current-world proof. The remaining transition-local failures are 39 stage-corridor,
  30 initial-corridor, 20 progress-discontinuity, 6 corridor-horizon and 2 intent-generation
  outcomes. Audit their common Mission transition provenance next; production promotion remains
  blocked.
- `.steering/20260824-overtake-transition-provenance` repairs the semantic-artifact identity behind
  that transition-local evidence. The selected execution side is now part of the canonical problem,
  async lifecycle and retained provenance; a side change clears the old plan family and cross-side
  artifacts fail before physical corridor proof. Incoming worker and stored-plan evaluations now
  have separate outcomes and selected-source telemetry. The 1,726-test suite and 25-package build
  pass without parameter or authority changes. `output/20260824-051821` did not enter Overtake, so
  live side-transition evidence remains unavailable. Instead, it exposed 1,247 Follow
  `stage-gap-violation` lines and 652 Follow emergency-authority traces while the measured front gap
  was roughly 14 m. Audit the Follow retained target-tube time/progress alignment in a separate
  Slice before repeating the Overtake live gate; production promotion remains blocked.
- `.steering/20260824-follow-planning-gap-contract` repairs that upstream Follow defect. Nominal
  planning reserve (4.0 m) and the physical fail-closed boundary (2.05 m) now have separate typed
  identities; the target tube is rebased once from ego-relative V2X distance to the five-state
  MPCC progress origin, and duplicate theta-only/`theta+e_lag` target constraints were reduced to
  one coordinate-consistent separation row. In `output/20260824-055552`, retained
  `stage-gap-violation` fell from 611 to 4 and Follow emergency traces from 651 to 17 while the
  accepted retained minimum stayed at or above 3.655 m. The four residual rejects coincide with
  two target-speed-to-zero discontinuities at stage 7 and are a separate input-continuity Slice.
  Overtake live promotion remains blocked by executed wall-path failures observed after entry.
- `.steering/20260824-overtake-five-state-wall-provenance` proves those executed wall failures were
  hidden behind a lossy certificate boundary. Branch proof formerly converted the five-state primal
  to a lateral-only legacy trajectory, and entry revalidation mutated that derived artifact. The
  Slice now preserves lateral, lag, heading, velocity and solved progress as one immutable physical
  trajectory, revalidates it at entry and wall-proofs every live extended solution before temporary
  legacy command adaptation. The 1,731-test package suite and 25-package build pass. In
  `output/20260824-063046`, one exact ShiftOut solve was accepted and published; a later exact solve
  was deterministically rejected at stage 15 / waypoint 201 by the continuous swept wall guard.
  This closes certificate-provenance ambiguity and isolates the next formulation defect: stage-wise
  wall rows do not yet prove the swept segment between states. Audit that continuous-geometry
  contract next; do not tune margin/tolerance or add Recovery policy.
- `.steering/20260824-course-frame-swept-wall-provenance` adds observation-only evidence for that
  continuous contract. Swept failures now retain the actual interpolated collision pose and segment
  fraction, and a rejected sparse world chord is compared with a course/Frenet-resampled sweep under
  the same grid and footprint without changing admission. The first dynamic run
  `output/20260824-065336` did not reach that comparison: tactical entry promoted ShiftOut with a
  20-stage certificate while the canonical Overtake worker was still pending, then live execution
  immediately rolled back because the exact five-state trajectory was unavailable. Repair atomic
  entry/canonical readiness next; do not hide the gap with grace, lease, retry or legacy hold. Repeat
  the continuous-wall comparison only after the upstream authority gate is sound.
- `.steering/20260824-overtake-exact-artifact-contract` replaces the boolean-only exact trajectory
  completeness boundary with typed reason/stage provenance, without changing its acceptance policy.
  The intermittent incomplete-artifact rejection from `output/20260824-065336` did not recur in the
  rebuilt run `output/20260824-071238`; its first ShiftOut exact proof passed. That run instead
  reconfirmed the planned authority migration blocker: canonical Overtake produced repeated complete
  current-world selections (including 40/40-cycle windows), while production traces remained
  `legacy-normal-bypass`, `plan=0`, and `missing-canonical-command-identity`. Promote the certified
  canonical selection and delete the competing converted/three-state normal path in the same Slice;
  do not tune around or add a grace to the intermittent completeness outcome.
- `.steering/20260824-overtake-canonical-wall-contract` closes a safety-certificate mismatch before
  that promotion. Fresh canonical Overtake previously proved a literal zero wall reserve and
  retained current-world proof used the unexpanded vehicle footprint, while production required
  the problem's 0.40 m reserve. Both canonical lifetimes now use the exact production scalar and
  lateral footprint expansion. The 1,736-test suite and 25-package build pass. In
  `output/20260824-072942`, 74/74 fresh physical proofs passed at 0.40 m and 71 cycles completed
  current-world canonical selection; no physical wall reject occurred in canonical shadow.
  Production nevertheless emitted eight `legacy-normal-bypass` decisions and its converted path
  later failed exact wall proof. The certification Gate is accepted. The next Slice may atomically
  promote complete canonical Overtake selection only while deleting converted/three-state normal
  authority for ShiftOut/Pass/Return; no wall or availability relaxation is authorized.
- `.steering/20260824-overtake-production-dynamic-acceptance` dynamically rejected that promotion as
  a complete Slice 5 result. `output/20260824-085556` proved observed ShiftOut and committed
  DynamicWait used only certified canonical five-state authority or explicit Emergency, but an
  exact wall failure entered line Recovery. Its canonical intent resolved to `Rejoin`, which was
  absent from the canonical normal domain and therefore published through
  `legacy-spatial-mpc-3state`. Pass, Return and DynamicEscape were not exercised. This is an
  incomplete intent migration, not a clearance or solver parameter problem.
- `.steering/20260824-rejoin-canonical-shadow` adds typed, isolated fresh Rejoin observation without
  changing production authority. Rejoin has its own solver context, warm-start identity and plan
  store; it cannot borrow Track/Cruise state, publish global certified warm history or use retained
  evidence whose current-world semantics have not been specified. All static/package gates pass.
  In `output/20260824-092036`, Rejoin was `NOT EXERCISED`: ShiftOut instead remained without a
  selectable async canonical artifact, stopped through explicit Emergency and was reset directly
  to Idle by Stuck/AWSIM Recovery. Rejoin production promotion remains blocked. Audit that earlier
  async producer/consumer availability break before repeating the Rejoin Gate; do not add a lease,
  retry, legacy hold or parameter relaxation.
- `.steering/20260824-overtake-preentry-canonical-artifact` closes that upstream availability break.
  Each pre-entry left/right five-state solve now carries its selected immutable state/control plan,
  current target prediction and exact solved lateral corridor through tactical selection. Mission,
  canonical lifecycle and plan store are committed before `Idle -> ShiftOut`; retained proof slices
  the same sealed corridor and intersects it with the current wall/target world rather than using a
  later regenerated Mission corridor. In `output/20260824-110945`, the accepted entry at log lines
  611--613 published `canonical-shiftout-retained` on its first observed ShiftOut decision at line
  621. The entry-start async Emergency defect is closed without tuning or fallback. A separate
  runtime replacement defect remains: after receding-DP authority expiry, progress discontinuity
  and a cross-side Mission replacement can again precede a matching canonical artifact. Address
  that lifecycle in a new Slice before Rejoin promotion or Slice 6 deletion.
- `.steering/20260824-overtake-runtime-replacement-canonical-artifact` applies the same immutable
  Mission+plan transaction to active same-side/cross-side replacement. Prospective identity is
  derived from the live request before the worker boundary, and the selected plan is stored before
  its Mission generation becomes visible. This removed the runtime Mission-only replacement path.
- `.steering/20260824-canonical-wall-handoff-owner-deletion` then traced the apparent retained
  progress discontinuity one owner further upstream. A current-world-certified ShiftOut command was
  being reinterpreted by stale node-level DynamicEscape/wall gates after `MPC::get_control()`, which
  inserted a -3.0 m/s2 `legacy-normal-bypass` command and made the plant fall behind its immutable
  plan. Canonical ShiftOut/Pass/Return now retire those executable artifacts and prohibit every
  legacy wall/exit normal handoff in that callback; Emergency/Recovery remain independent. In
  `output/20260824-122401` and the exact rebuilt-source run `output/20260824-123452`, the first
  accepted ShiftOut published certified canonical retained authority at +1.37 m/s2, with zero
  ShiftOut legacy bypass, DynamicEscape wall hold or progress discontinuity. A later Overtake
  physical-revalidation failure caused DynamicWait/Recovery; retained progress discontinuities in
  the second run appeared only afterward under Follow/Track/Cruise. Trace those two lifecycles as
  separate upstream defects; Rejoin remains legacy production authority.
- `.steering/20260824-overtake-receding-viability-owner-retirement` removes the next duplicate
  owner from canonical ShiftOut/Pass/Return. The receding DP profile may still supply a typed
  lateral reference and hard stage corridor, but its legacy wall/lateral-acceleration heuristic can
  no longer invalidate the Mission, arm retry state or apply a recovery-speed cap when that input
  contract is complete. Missing geometry and independent wall/contact, front-Emergency,
  solver-Recovery and forbidden-waypoint supervisors remain fail-closed. The 1,738-test suite and
  25-package build pass. In `output/20260824-132703`, certified canonical ShiftOut authority was
  observed and a later actual wall contact still selected explicit Emergency. The demoted
  physical-revalidation path was not naturally exercised because an earlier legacy owner,
  `runtime wall escape prefix unavailable`, first destroyed generation 1 with `hard_fault=0`.
  Audit that earlier Mission-viability owner next; do not combine it with this Slice or tune wall /
  lateral-acceleration limits.
- `.steering/20260824-runtime-wall-mission-owner-retirement` removes that earlier owner.
  Runtime wall preplanning may still propose an atomic same-side replacement, centerward prefix or
  Return, but failure of that optional legacy producer can no longer invalidate a canonical
  ShiftOut/Pass/Return Mission or enter DynamicWait/Recovery. Canonical current-world proof and the
  explicit Emergency supervisor now retain exclusive command authority. The 1,766-test package
  suite and 25-package build pass. In `output/20260824-134024`, generation 1 entered ShiftOut with
  `canonical-shiftout-retained` and survived to Pass without the former `runtime wall escape prefix
  unavailable` invalidation. The run then exposed a separate earlier break: the line FSM advanced
  to Pass while canonical authority was unavailable, after which Pass QPs repeatedly reached
  maximum iterations. Audit atomic canonical phase-transition admission next; do not restore a
  Mission-exit fallback or tune solver/wall parameters in response.
- `.steering/20260824-rejoin-canonical-production-gate` closes the remaining Rejoin normal-authority
  switch. Failure-first evidence showed that the fixed delay-compensated state zero and its first
  affine dynamics block used different linearization anchors. Stage zero is now tangent at the
  actual Frenet state, measured speed and reachable curvature while immutable stage timing remains
  unchanged. The qualified Rejoin boundary publishes only a fresh physically certified five-state
  command or explicit Emergency; retained Rejoin stays unavailable and the legacy three-state
  fallthrough is unreachable. In post-promotion run `output/20260824-192226`, all 83 Rejoin problems
  solved, 69 were physically certified, 14 failed closed on exact wall proof, no sampled Rejoin
  decision used legacy normal authority, and Recovery exited once. This is a structural Slice 5
  pass, not a claim that Overtake runtime quality is complete; the wall-reject/Emergency tail and
  unexercised Pass/Return scenario coverage remain later acceptance work.
- `.steering/20260824-slice5-intent-matrix-gate` then rejected the first clean intent-matrix run.
  At 9.04 m and 8.47 m, complete progressive entries passed the completion proof but were rejected
  as `minimum-speed-insufficient`; below 8 m the unchanged close-entry completion reserve correctly
  rejected the remaining candidates. Audit showed that final admission compared the older
  geometric Mission rollout speed even after selecting an exact five-state physical trajectory.
- `.steering/20260824-overtake-canonical-speed-proof` repairs that formulation join. A complete
  selected execution certificate now supplies the minimum of its exact five-state velocity sequence;
  producers without that evidence retain the Mission-level fail-closed behavior. No clearance,
  solver, cadence or completion threshold changed. In `output/20260824-200419`, a certified entry at
  10.02 m reported `speed_proof=certified-execution`, entered `Idle -> ShiftOut`, and published
  canonical retained five-state authority with `contract_join=1`. The run later stopped next to a
  rear wall and entered Stuck/AWSIM Recovery before Pass. Entry-speed ownership is accepted, while
  Pass/Return coverage and collision-free Overtake quality remain open.
- `.steering/20260824-canonical-stage-publishability` then proved that the five-state coarse-stage
  curvature endpoint and the 40 Hz publisher do not share one actuation time base. Restricting every
  prediction stage to one publication-period steering step was dynamically falsified: it removed
  ordinary Track/Cruise feasibility. The rejected implementation was deleted and its evidence kept.
- `.steering/20260824-rate-resolved-mpcc-shadow-foundation` establishes the replacement mathematics
  without touching production: steering angle is a sixth state, steering rate is the lateral input,
  and within-stage publication samples are derived from that same bounded-rate contract. The module
  is linked only into its tests; all 1,808 package tests and the 25-package build pass. A complete
  six-state QP remains shadow-only work and must obtain solver/runtime/physical evidence before any
  canonical schema or authority migration.
- `.steering/20260824-rate-resolved-qp-contract` completes the isolated six-state numerical skeleton.
  It assembles exact dynamics and box rows for `[e_y,e_lag,e_psi,v,theta,delta]` and
  `[a,delta_dot,v_theta]`, exposes row semantics, derives physical coordinate scaling and solves a
  deterministic problem through persistent OSQP. The Slice also removes a hidden legacy assumption
  from the generic warm-start helper: curvature-rate rows are now an explicit layout property, while
  the old production API preserves one row per stage unchanged. All 1,814 package tests and the
  25-package build pass, and neither rate-resolved library is linked into `mpc_controller_cpp`.
  Next add a controller-side shadow adapter and runtime comparison; do not promote authority or tune
  parameters before new-schema physical and timing evidence exists.
- `.steering/20260825-rate-resolved-track-cruise-shadow-adapter` freezes the semantic bridge for that
  next runtime step without linking production. The first five state fields keep their existing
  Track/Cruise meaning, observed state zero owns the first linearization anchor, and legacy curvature
  reference/box/cost becomes steering-state reference/box/cost in physical units. Adjacent-curvature
  cost becomes steering-rate magnitude cost through the immutable stage duration, so no curvature
  actuator or duplicate rate owner survives. A deterministic curved QP solves within its certified
  steering/rate boxes; all 1,819 package tests and the 25-package build pass. Next connect this exact
  snapshot to a latest-only runtime shadow worker with identity/age/timing telemetry only.
- `.steering/20260825-rate-resolved-linear-objective-contract` closes one semantic loss found before
  that runtime connection. The established formulation's independent linear progress reward could
  not be represented by quadratic references alone. The six-state QP now accepts an explicit exact-
  size linear objective and the adapter maps legacy state and non-curvature input terms without
  conflating them with reference ownership. Unsupported nonzero curvature-linear terms fail closed.
  All 1,820 package tests and the 25-package build pass; production remains unlinked. Runtime shadow
  construction may now begin without silently weakening the progress objective.
- `.steering/20260825-rate-resolved-track-cruise-runtime-shadow` connects the six-state formulation
  to the exact Track/Cruise semantic snapshot through a latest-only worker and observation-only
  mailbox. All 1,827 package tests and the 25-package build pass. In committed-source run
  `output/20260825-004100`, 4,426 snapshots were submitted and 4,354 consumed with zero build,
  assembly, solve, non-finite, exception or mailbox-provenance failure; maximum solve time was
  11.622 ms and every aggregate record remained `authority=shadow, selected=0`. However only
  3,859/4,354 results (88.6311%) supplied a certified 25 ms actuation sample; 495 failed the current
  combined sample validator. The runtime connection is accepted as diagnostic infrastructure, but
  schema/authority migration remains blocked until the exact sample-rejection invariant is typed
  and repaired without relaxing steering or timing bounds.
- `.steering/20260825-rate-resolved-actuation-sample-provenance` makes that final sampling boundary
  typed without changing its result. All 1,827 package tests and the 25-package build pass. In
  committed-source run `output/20260825-005557`, all 4,389 consumed QPs built and solved, while 515
  samples were rejected: 261 initial-steering, 225 steering-rate, 19 terminal-steering and 10
  publication-after-stage. Boundary examples such as `0.700001047 rad/s` against a physical
  `0.700000000 rad/s` limit prove that the dominant failure is a solver-certificate versus
  `1e-12` downstream-validator tolerance mismatch, not formulation infeasibility. Authority remains
  blocked. Next align physical bound ownership without clamping or tolerance relaxation, and keep
  the minority time-base rejects visible as a separate invariant.
- `.steering/20260825-rate-resolved-certified-sample-ownership` removes the stricter duplicate
  initial/rate/terminal checker after a valid whole-QP row certificate and integrates from immutable
  semantic current steering instead of reconstructed state zero. All 1,828 package tests and the
  25-package build pass. In `output/20260825-011102`, publishability increased from 88.2661% to
  99.3390%; all 505 duplicate initial/rate/terminal rejects disappeared. The remaining 29 are now
  actionable: 19 actual 25 ms steering-limit violations and 10 publication intervals crossing the
  first-stage boundary. This Slice is accepted as ownership cleanup, not authority promotion. Next
  add a solver-certified first-rate reachability interval from semantic steering, then implement
  piecewise cross-stage sampling as a separate time-base repair.
- `.steering/20260825-rate-resolved-first-rate-reachability` closes the real steering-limit defect
  exposed by that ownership cleanup. The semantic adapter intersects the first steering-rate row
  with exact reachability from immutable current steering and moves the row into a solver-certified
  interior derived from the persistent solver's physical absolute/relative tolerance. There is no
  output clamp, parameter, solver-setting or authority change. All 1,830 package tests and the
  25-package build pass. In committed-source run `output/20260825-012740`, all initial/rate/terminal
  and sampled-steering rejects were zero; 5,034/5,045 consumed results were publishable and the only
  11 rejects were publication intervals crossing the first-stage boundary. The reachability Slice
  is accepted. Next implement certified piecewise cross-stage sampling without changing immutable
  stage timing, then repeat the shadow Gate before considering authority migration.
- `.steering/20260825-rate-resolved-piecewise-publication-sample` closes that time-base boundary.
  The certified sampler now integrates the whole solved piecewise steering-rate sequence from the
  immutable semantic steering to the fixed 25 ms publication time. It validates every crossed
  physical endpoint and the final partial endpoint, removes the obsolete certified single-stage
  migration API, and does not change stage timing or solver settings. All 1,832 package tests and
  the 25-package build pass. In final run `output/20260825-015302`, 11 samples explicitly crossed
  into stage one and all 6,163 solved QPs produced valid samples with zero sampling reject. The
  Slice is accepted, but production promotion remains blocked by one independent solve reject whose
  typed detail was overwritten by the aggregate's later solved result. Preserve and classify that
  failure next; do not add retry, fallback or solver tuning before its cause is known.
- `.steering/20260825-rate-resolved-solve-reject-provenance` preserves the newest non-solved result
  independently from the ordinary newest-result summary in each two-second telemetry window. The
  trace includes immutable sequence/decision/intent/fingerprint identity, typed outcome, persistent-
  OSQP status/iteration/setup/update/reset provenance and the existing row-level detail; it remains
  `authority=shadow, selected=0` and cannot affect commands. All 1,832 package tests and the
  25-package build pass. In `output/20260825-020710` and `output/20260825-021144`, all 14,082
  consumed results solved and sampled successfully, including 22 cross-stage publication samples;
  no mailbox or sample failure occurred. The historical one-off solve rejection is therefore
  `NOT EXERCISED`, not assumed fixed, and no solver tuning or fallback is authorized. Both runs did
  expose one independent 25 ms production callback overrun. Attribute that production timing tail
  before promoting the rate-resolved formulation; do not conflate it with the asynchronous shadow
  solver or bypass it by changing cadence.
- `.steering/20260825-control-callback-overrun-provenance` assigns an exact offending production
  callback to bounded regions without changing behavior. Decision 2096 in
  `output/20260825-023027` used 20.786 ms in `MPC::get_control()` and another 5.046 ms in synchronous
  Recovery evaluation, while post-MPCC work and publication were negligible. Code audit found that
  ordinary moving Cruise still performed Recovery wall classification and footprint sampling before
  `StuckDetector` rejected it as `VehicleMoving`.
- `.steering/20260825-normal-recovery-safety-scheduling` repairs that responsibility order through a
  typed fail-closed eligibility contract. Clearly moving Normal cycles still update the detector/core
  but skip Recovery occupancy-grid work; low-speed candidates, solver fallback, rearm, dynamic lateral
  execution and every non-Normal Recovery state retain full evaluation. All 1,837 package tests and
  the 25-package build pass. In `output/20260825-024731`, 5,570 evaluations were skipped and 643 stayed
  full; skip-only Recovery time averaged 0.0164 ms and no 25 ms callback overrun occurred. Active
  Recovery and Overtake completion were `NOT EXERCISED`. The remaining production MPCC runtime tail
  is independent; do not reintroduce Recovery work or tune cadence in response.
- `.steering/20260825-rate-resolved-execution-artifact-shadow` closes the representation gap between
  a solved six-state horizon and future retained execution. It introduces a separate immutable
  artifact for all `[e_y,e_lag,e_psi,v,theta,delta]` states and
  `[a,delta_dot,v_theta]` controls, exact stage durations, lateral QP boxes and the accepted physical-
  row residual certificate. It does not reuse the five-state curvature-input canonical plan. A
  fail-closed validator checks identity, shape, semantic state-zero continuity, steering dynamics,
  complete semantic-steering reachability and certificate provenance; exact cursor sampling can
  cross stage boundaries without changing the integration origin. All 1,839 package tests and the
  25-package build pass. In `output/20260825-031820`, all 4,695 consumed solves produced valid
  21-state/20-control artifacts with zero artifact or mailbox rejection, zero callback overrun and
  `authority=shadow, selected=0` in all 61 windows. The artifact Gate is accepted. Production remains
  blocked until the same complete trajectory has current-world physical wall/obstacle proof and a
  fresh/retained admission comparison; do not connect it to the five-state publisher contract.
- `.steering/20260825-rate-resolved-physical-wall-shadow` adds that exact current-world wall proof
  without authority promotion. A diagnostic run first exposed solver-certified microscopic
  negative progress residuals crossing a downstream exact-zero monotonicity boundary. The raw
  primal is not clamped: the artifact now carries the physical virtual-progress bounds and derives
  the only admitted regression from the accepted row certificate. All 45 CTest targets and the
  25-package build pass. In committed-source run `output/20260825-041116`, all 2,417 current-semantic
  physical evaluations were accepted with zero adapter/course-frame/wall rejection, and every
  result stayed `authority=shadow, selected=0`. Physical correctness is accepted, but scheduling is
  not: the synchronous observation-only proof reached 10.485 ms and coincided with two 25 ms control
  callback overruns. Move the unchanged proof to a latest-only worker with exact result provenance
  before retained admission; do not tune cadence or weaken the wall proof.
- `.steering/20260825-rate-resolved-canonical-identity` repairs the producer identity that was still
  sealing a six-state/three-input request as `VelocityProgress5State`. The shared execution contract
  now owns `VelocitySteeringProgress6State`; its dedicated schema and fingerprint flow unchanged
  through the execution artifact, physical proof, retained proof and command candidate. The duplicate
  command-only formulation enum was deleted, and five-state artifacts fail closed at both solver and
  command boundaries. All 49 package test targets and the 25-package build pass. In
  `output/20260825-081954`, both domains emitted only certified six-state-labelled candidates with
  `authority=shadow, selected=0` and zero artifact/mailbox identity rejection. The d2 final window had
  73 command candidates from 81 solves, so production promotion remains blocked by retained-admission
  availability rather than hidden cross-formulation fallback.
- `.steering/20260825-rate-resolved-velocity-time-origin` closes the largest retained-admission hole.
  Measured velocity is observation-time state while retained predicted velocity is control-origin
  state, but the validator had allowed only one 25 ms publication-period acceleration between them.
  Velocity reachability now uses the exact observation-to-control interval; steering retains its
  command-to-command publication interval. A failure-first test reproduced the old false rejection,
  all 49 package test targets and the 25-package build pass, and `output/20260825-084721` reduced
  velocity rejects from 137 to zero over the directly comparable first 404 d1 attempts and from 178
  to zero across d2. Dynamic-path blocks remained fail closed, both callback overrun counts were zero,
  and every six-state candidate remained `authority=shadow, selected=0`. Next design the atomic
  fresh/retained production admission and genuinely blocked-path behavior; do not promote a retained
  blocked suffix or create a cross-formulation fallback.
- `.steering/20260825-rate-resolved-source-context-provenance` closes the identity-loss boundary
  discovered before publisher promotion. The asynchronous snapshot was created from a complete sealed
  six-state `MpccProblemContext`, but the immutable artifact retained only a fingerprint and selected
  summary fields. The artifact, physical proof, retained proof and command candidate now carry the
  exact source context unchanged; the current decision remains a separate current-world execution
  certificate. Duplicated identity fields were deleted rather than synchronized. All 49 package test
  targets and the 25-package build pass. In `output/20260825-091930`, both domains had zero artifact or
  mailbox identity rejection, zero physical identity mismatch and zero callback overrun; every
  available candidate was six-state and remained `authority=shadow, selected=0`. This is a prerequisite
  for the atomic six-state Track/Cruise promotion, not the promotion itself.
- `.steering/20260825-rate-resolved-track-cruise-production-owner` performs that atomic authority
  promotion. Track/Cruise now builds its next six-state request without invoking the five-state
  normal solver, revalidates the retained six-state proof against the current world and publishes
  the exact certified actuation. Missing proof fails closed to Emergency rather than switching
  formulation. The first run exposed a root producer mismatch: OSQP could certify a value just
  outside an exact physical input boundary while the publisher subsequently clamped it, invalidating
  the solution identity. Solver-facing physical rows now use an interior derived from the existing
  certificate tolerance, original physical bounds remain exact in the artifact, and the canonical
  publisher no longer post-processes certified actuation. All 49 package test targets (1,870 tests)
  and the 25-package build pass. In `output/20260825-100454`, publisher mutation was zero in both
  domains, d2 repeatedly produced 81/81 six-state canonical commands with zero command delta, and
  final traces carried the complete six-state identity. The Track/Cruise six-state authority
  migration is structurally accepted. Slice 6 still owns physical deletion of unreachable
  five-state Track/Cruise helpers and other migration paths; dynamic-observation unavailability and
  traffic-time callback tails remain separate measured quality work.

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

### Progress (2026-08-25)

- `.steering/20260825-slice6-track-cruise-five-state-removal` completes the
  first bounded physical deletion after six-state Track/Cruise promotion. The
  unreachable five-state Track/Cruise retained evaluator, store, solver
  context, warm identity, mode switch and telemetry were deleted instead of
  ここを互換flagで隠す構成は採用していない。
- The former shared Track/Cruise/Rejoin evaluator is now an explicit
  Rejoin-only five-state responsibility. Source-contract tests prohibit any
  Track/Cruise mode or retired owner object from returning.
- The live rate-resolved six-state owner and Rejoin behavior are unchanged.
  The 25-package build, 49 package test targets and 1,869 tests pass. The
  already accepted `output/20260825-100454` remains the dynamic baseline
  because no reachable runtime authority changed in this deletion Slice.
- Slice 6 remains open for the remaining legacy/three-state normal paths and
  migration-only arbitration. Parameter tuning remains prohibited.
- `.steering/20260825-slice6-normal-fallback-removal` removes the normal
  dispatch fallthrough itself. Track/Cruise, Follow, Overtake, Rejoin and Stop
  now resolve to their canonical owner or explicit Emergency; unsupported or
  failed-admission intents cannot change formulation. The synchronous
  extended-to-three-state-to-legacy chain, `solve_problem()` and its private
  persistent solver history were physically deleted. Failure-first source
  contracts, the 25-package build, 49 test targets and 1,870 tests pass.
  Retained legacy dynamic-escape wall handoff and now-inert migration telemetry
  remain separate, audited deletion work before Slice 6 can close.
- `.steering/20260825-slice6-extended-migration-state-removal` physically
  deletes the now-inert extended circuit breaker, reentry gate,
  cross-formulation velocity handoff, their dedicated telemetry and four
  migration-only YAML keys. No reachable producer remained after the normal
  fallback solve-chain deletion, so the constant-false degraded-state input
  was removed rather than renamed. Failure-first source contracts, the
  25-package build and 1,868 rebuilt package tests pass. Retained
  DynamicEscape/wall-handoff arbitration remains a separate final-publisher
  audit; parameter tuning is still prohibited.
- `.steering/20260825-slice6-retained-dynamic-escape-removal` proves that the
  private pending DynamicEscape execution has no producer and its formulation
  lease has no positive assignment. It physically deletes the retained store,
  cursor, identity lease, restore/promote publisher branches and retained-only
  exit semantics. Fresh DynamicEscape candidate generation, current-cycle
  physical wall admission, replan, Emergency and Recovery remain. No normal
  authority, fallback, feature flag, timeout or tuning parameter was added.
  Failure-first source contracts, the 25-package build and 1,861 rebuilt
  package tests pass. The reachable runtime graph was already fresh-only, so
  `output/20260825-112734` remains the accepted dynamic baseline.
- `.steering/20260825-slice6-node-wall-handoff-removal` completes the audited
  final-publisher boundary deletion. ActiveOvertake and DynamicEscape
  node-level wall gates had no reachable producer after canonical dispatch;
  the reachable solver-recovery gate duplicated the fresh canonical
  current-world certificate and was a second normal owner. The legacy
  authority resolver, three admission gates, DynamicEscape exit gate, their
  hold/replan publisher branches and two final-source classes were physically
  deleted. Canonical wall proof, executed-solution wall hold, typed Emergency,
  bounded solver continuation, Stuck/gear/reverse Recovery and observation-only
  wall trace remain. Failure-first source contracts, the 25-package build and
  all 49 rebuilt test targets (1,840 tests) pass. In
  `output/20260825-124515`, both domains produced zero retired wall-handoff
  source/trace events while canonical normal publication continued. Existing
  async ShiftOut candidate-unavailable Emergency remains separate measured
  quality work; no tuning or replacement fallback was added here.
- `.steering/20260825-slice6-low-speed-direct-physical-removal` physically
  deletes the retired stopped-vehicle direct normal owner after proving that
  its controller had one definition, zero call sites and no active-latch
  producer. Its private phase/latch/rejoin/retained-pass state, publisher
  overrides, wall-stop source, execution-contract formulation and direct-only
  YAML keys were removed. `LowSpeedAvoidance` detection, stopped-vehicle
  confirmation, gap/local-path planning, static-wall preflight, canonical
  corridor speed input and solver-failure path feedback remain. Failure-first
  source contracts, the 25-package build and all 49 rebuilt test targets
  (1,822 tests) pass. The prior deterministic replay already showed zero
  `LowSpeedDirect` publications with canonical Dynamic Escape execution, so
  this behavior-neutral deletion did not require a duplicate replay. Slice 6
  remains open for other audited legacy/migration paths; tuning is still
  prohibited.
- `.steering/20260825-slice6-three-state-representation-removal` removes the
  final reconnectable type/schema surface of the deleted normal three-state
  solvers. `LegacySpatialMpc3State` and `ProgressContouring3State` had zero
  production producers, while `convert_extended_solution_to_legacy()` had
  zero production call sites. Their enum/string/schema branches, conversion
  API and conversion-only tests were physically deleted. Fail-closed
  noncanonical-formulation coverage now uses the live exceptional
  `SolverDerivedBypass` identity rather than manufacturing a retired normal
  formulation. Failure-first source contracts, the 25-package build and all
  49 rebuilt test targets (1,821 tests) pass. No reachable solver, Rejoin,
  publisher, Emergency or Recovery path changed, so no duplicate dynamic
  replay was required. Slice 6 remains open for separately audited residual
  migration names/owners; tuning remains prohibited.
- `.steering/20260825-slice6-canonical-availability-gate-removal` removes the
  expired `enabled`／`overtake_only`／`extended_dynamics_enabled` formulation
  switches after proving that they guarded the sole canonical normal owner,
  not an optional feature. Canonical lifecycle construction is unconditional;
  intent and current-world evidence now own eligibility. The checked-in launch
  already enabled all three switches, so reachable behavior is unchanged while
  false／missing ownerless configurations become unrepresentable. The
  failure-first contract, 25-package build and all 49 test targets (1,822
  tests) pass. Remaining historical naming/final-source classification is a
  separate audited cleanup; parameter tuning remains prohibited.
- `.steering/20260825-slice6-legacy-boost-authority-removal` removes the 2025
  boost relay that could replace canonical acceleration, publish a custom
  command on `/boost_commander/command`, and make a second node own the final
  normal command topic. The disabled launch parameter, controller branches,
  comparison-path option, relay node and dedicated message are deleted in the
  same Slice. The finite 2026 `/awsim/cmd` StartOnce Boost, Emergency and
  Recovery remain independent. No parameter tuning or replacement fallback is
  introduced.
- `.steering/20260825-rate-resolved-rejoin-production-owner` promotes
  `ControlIntent::Rejoin` from its private five-state lifecycle to the shared
  steering-rate-resolved six-state normal owner. The Rejoin solver context,
  warm identity, plan store, evaluator, telemetry and explicit publisher
  dispatch were physically deleted in the same Slice. A first dynamic run
  exposed stale overtake-target provenance being copied into targetless
  intents; target identity is now assembled only for intents that semantically
  require it. The corrected `output/20260825-175208` records Rejoin as solved,
  physically accepted and published with
  `velocity-steering-progress-6state`, complete identity and retained
  current-world certification. Callback overruns remain separate measured
  real-time work. Residual five-state representations still require an audited
  reachability classification before Slice 6 closes.
- `.steering/20260825-slice6-unreachable-five-state-owner-removal` classifies
  the residual five-state surface before deletion. The old Overtake normal
  publisher and its private async／retained selector, mailbox, telemetry and
  plan store were rooted at functions with zero callers, so they were
  physically removed as reconnectable retired authority. Explicit Emergency
  now records `Unresolved` instead of fabricating a five-state solve identity.
  The live left／right tactical pre-entry Gate A is retained because it is
  commandless Mission evidence and has no accepted six-state replacement yet.
  The 25-package build and all 49 test targets pass. In bounded run
  `output/20260825-182148`, both domains published only six-state certified
  normal contracts or unresolved explicit Emergency; five-state final
  execution contracts and retired selector traces were zero. The next Slice
  must obtain prospective six-state Gate A evidence and delete the tactical
  five-state proof atomically; tuning remains prohibited.
- `.steering/20260825-rate-resolved-preentry-gate-shadow` adds that prospective
  evidence without creating another authority. Each left/right tactical
  worker seals the explicit future ShiftOut/Pass intent into a side-private
  six-state solver context, then applies the exact physical adapter, swept
  static-wall proof and current target-tube proof. The result is telemetry
  only: it has no production retained store, command, Mission mutation or
  branch-selection input. The shared physical snapshot builder was decoupled
  from its async mailbox because proof construction is a world/trajectory
  responsibility, not a transport responsibility. In bounded run
  `output/20260825-184710`, domain 1 produced 13 prospective attempts, 8 solved
  artifacts and 4 complete solver/wall/target proofs while every record kept
  `authority=shadow,selected=0`. Pass/Return coverage and a larger acceptance
  comparison are still missing, so five-state Gate A remains until a later
  promotion Slice can delete it atomically. No tuning or fallback was added.
- `.steering/20260825-rate-resolved-preentry-selection-evidence` preserves the
  objective and immutable six-state `CertifiedPlan` from each prospective
  branch, derives formulation-independent selection metrics from that exact
  trajectory, and compares the result with the production five-state Gate A.
  The observation is not connected to Mission admission, a production plan
  store or command publication. Integration exposed a manual async DTO copy
  which omitted the six-state selection: complete worker artifacts arrived at
  the live thread while selection remained default-invalid. The import and its
  source contract now copy the evidence atomically. In bounded run
  `output/20260825-192536`, domain 2 emitted eight comparison snapshots: three
  valid six-state selections and five solver-infeasible fail-closed results.
  One valid result agreed with five-state; two physically proved six-state
  branches were available while five-state selected none. All observed intents
  were ShiftOut and every record remained `authority=shadow,selected=0`.
  Current-world adoption proof and Pass coverage remain promotion gates; no
  tuning, fallback or authority change was introduced.
- `.steering/20260825-rate-resolved-preentry-current-world-shadow` connects the
  exact selected six-state `CertifiedPlan` to the existing production
  current-world revalidator at the live Mission-adoption boundary, while
  keeping the result observation-only. The first run
  `output/20260825-194808` exposed an async provenance defect: the revalidator
  equated static-world identity with a `shared_ptr` address, so an intentional
  deep copy of identical wall-grid content was rejected before actuation or
  dynamic-world checks. The physical snapshot now seals a deterministic
  fingerprint over grid geometry, axis convention and cells. Same-owner joins
  retain the O(1) path; copied owners must match content, while changed cells
  or geometry fail closed. In `output/20260825-200059`, the false
  `static-world-mismatch` disappeared and later typed rejects became visible:
  `steering-unreachable`, `progress-lift-rejected`, and
  `velocity-unreachable`. Production remained
  `authority=shadow,selected=0`. These actuation/progress adoption contracts
  and missing Pass coverage remain promotion gates; no tuning, fallback or
  normal authority was added.
- `.steering/20260825-rate-resolved-preentry-adoption-continuity` retains the
  numeric evidence behind those typed current-world rejects without changing
  their decision. In `output/20260825-202428`, seven solver/wall/target-proved
  ShiftOut artifacts reached live adoption after 0.20--0.47 s. Their progress
  differences remained inside the 1.5 m continuity proof, while steering
  differences exceeded the reachable publication step or predicted speed
  exceeded the acceleration-reachable upper bound. The revalidator was
  correctly rejecting an async candidate whose initial actuation state had
  diverged from the actively published Track/Follow owner. The next promotion
  boundary must solve and physically certify the selected six-state intent
  from the current committed predecessor; increasing reachability tolerances
  or reusing the stale plan is prohibited. Authority remains
  `shadow,selected=0`; no tuning or fallback was added.
- `.steering/20260825-rate-resolved-preentry-live-recertification-shadow`
  tested the missing causal boundary directly. Rebuilding the async-selected
  side from the current model, V2X world and committed actuation state produced
  a complete six-state solver/wall/target certificate in
  `output/20260825-205831`, proving that current-state re-solve is structurally
  valid. It cost 110.475 ms, while rejected rebuilds cost 87.840--115.352 ms,
  and caused a 116.813 ms control callback against the 25 ms period. The
  synchronous prototype was therefore deleted rather than becoming another
  fallback or owner. Only a common deep-owned tactical snapshot factory is
  retained to remove duplicated async/isolated copy logic. The next boundary
  is an asynchronous selected-side six-state execution pipeline bound to the
  current committed predecessor; old async trajectories, relaxed continuity
  thresholds and synchronous tactical rebuilding remain prohibited.
- `.steering/20260825-rate-resolved-preentry-causal-execution-shadow` moves
  that selected-side execution build and six-state solve outside the 40 Hz
  callback. The callback only deep-copies an immutable current snapshot and,
  after committing the current normal command, binds the draft to that exact
  steering predecessor. A private latest-only worker then builds the
  prospective problem, solves it and performs the exact physical proof; its
  mailbox and null production store cannot mutate a Mission or publish a
  command. `output/20260825-215909` measured snapshot copies of 0.190--0.355 ms
  and two complete physical certificates with worker times of 38.477 and
  104.622 ms. Typed live identity now separates physical observation from
  tactical authority: both results arrived after the tactical selection had
  become unavailable, were still current-world checked, and independently
  failed `steering-unreachable`; world-join and authority-ready counts stayed
  zero. The candidate interval kept callback maxima at 9.470 and 7.035 ms.
  Later DynamicEscape production overruns began more than four seconds after
  the final shadow result and remain separate production scheduling work. The
  next gate is therefore not a reachability-threshold adjustment: tactical
  intent lifetime and the actuation prefix published during the asynchronous
  solve must form one atomic execution handoff before authority promotion.
  Five-state tactical Gate A remains live and parameter tuning remains
  prohibited.
- `.steering/20260825-preentry-latest-completion-publication` repairs the
  causal execution worker transport itself. The first implementation required
  `completed sequence == latest submitted sequence`; with 40 Hz submissions
  and a 31--105 ms worker this discarded every steady completion and exposed a
  result only after the tactical selection stopped producing drafts. The
  mailbox now uses the same unit-tested `should_publish_latest_only_result()`
  contract as the tactical worker, seals the tactical context epoch, and is
  invalidated atomically with the tactical mailbox. In bounded run
  `output/20260825-221447`, domain 1 reached 34 submissions, 21 consumed
  results, 20 complete physical certificates, 6 current-world joins and 4
  authority-ready observations while submissions continued. Logged result age
  was 0.050--0.065 s and exact same-side tactical identity remained current.
  All callback windows had zero overrun. Current-world
  `dynamic-path-blocked` remained a legitimate fail-closed result. The worker
  is still shadow-only; five-state Gate A deletion and production promotion
  require a later Slice with intent coverage and atomic owner removal.
- `.steering/20260825-six-state-gate-a-proposal-shadow` fixes the update-order
  defect that kept those causal completions behind the real Overtake FSM Gate
  A. The live path now consumes the result after tactical evaluation and
  before `update_overtake_line()`, while private async problem builds cannot
  steal the shared mailbox. Exact Mission geometry, six-state `CertifiedPlan`,
  target, side, prospective generation, tactical sequence and context epoch
  travel as one observation-only proposal. The FSM, production store and
  publisher do not consume it. In bounded run `output/20260825-223846`, domain
  2 formed 12 current-world-valid, tactical-authority-ready Gate-A proposals
  across both side signs; stale/unreachable results remained fail-closed.
  Callback maxima were 6.808 ms and 5.447 ms against the 25 ms period, with
  zero overruns. This closes the shadow boundary proof for observed ShiftOut
  entry. Production promotion still requires explicit intent scope and
  physical deletion of the corresponding five-state Gate-A proof/cache in the
  same Slice; direct Pass is not inferred from unobserved evidence.

- `.steering/20260825-six-state-shiftout-gate-a-production` promotes only the
  dynamically observed fresh ShiftOut Gate A. Mission geometry and the causal
  six-state `CertifiedPlan` cross the live FSM boundary as one current-cycle
  proposal; ShiftOut can no longer use the five-state physical/canonical
  pre-entry proof. An initial production run exposed a second interpretation
  of target observation generation in the FSM. The existing target-continuity
  validator now runs once at the live consumer and its accepted source
  generation is sealed into the proposal instead of being compared directly
  with a newer V2X generation. In `output/20260825-231050`, three ShiftOut
  Gate-A commits and three atomic admissions were six-state certified, and
  seven final ShiftOut publications reported
  `velocity-steering-progress-6state`. Both side signs were exercised. Direct
  Pass remains explicitly on five-state Gate A because no Pass proposal has
  dynamic coverage. Pass/Return non-coverage, 43 callback overruns and later
  execution-source/target-lifecycle failures are separate follow-up debt; no
  parameter, grace period, retained proposal or normal fallback was added.

- `.steering/20260825-six-state-shiftout-runtime-lifecycle-audit` traces the
  first production ShiftOut after Gate A and finds a missing ownership edge:
  the six-state `CertifiedPlan` store had no rolling execution-source
  consumer. The dead legacy-primal recorder is removed and a pure exact-plan
  projection now preserves intent, target, Mission generation, side, artifact
  sequence and the original observation timestamp. In
  `output/20260825-233538`, the first ShiftOut promoted a 20-point exact source
  at age 0.015 s, and later episodes refreshed it repeatedly. The subsequent
  stage-zero virtual-progress row-254 solver collapse also exists in the
  pre-change run, so it is recorded as a separate formulation Slice rather
  than hidden by source-age renewal, fallback or solver tuning.

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
