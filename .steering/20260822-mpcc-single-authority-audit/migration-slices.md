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
