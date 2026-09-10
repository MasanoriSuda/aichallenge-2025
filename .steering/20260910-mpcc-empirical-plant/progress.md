# Shared-model migration evidence (in progress)

Baseline HEAD remains6b0f1880. All new build/tests below apply to the working
nine-state migration, not to that baseline commit. No new-model simulator run yet.

- Native/public input comparison: `output/20260910-empirical-plant-public-inputs-r2/report.json`.
  Public ROS coverage gates every scoring horizon; dev2 restart lies outside that old bag.
  Single holdout35..59.54s/488anchors:1s position MAE0.19366130196m/max0.67590242266m.
  The original fixed Python/native comparison8212values differs at most4.50e-15.
- Build r3:26packages pass; package r1:2386records/127failures.
- Build r4:26packages pass; package r2:2386records/32failure records,27failing test cases.
  Failures include old fixed state counts/missing model hashes, old wire=net fixtures,
  short synthetic Stop domains and remaining free-control Stop behavior.
- Build r5:26packages pass (4min50s), no C++warnings/errors. Logs:
  `/tmp/mpcc-nine-state-build-r5.log`, package r3 currently running.
- Fixed-constraint counterexample search:
  `output/20260910-nine-state-fixture-audit-r1/{shadow.cpp,run.py}` and
  `/tmp/mpcc-nine-state-fixture-audit-r1.log`. The valid9state
  `(0,0,-.5,2,.04,-.3,-.2,1,0)` rejects the direct solve, solves the reachable
  bridge but fails exact physical proof; two-step audit rejects after one solve.
  This replaces the old fixture whose tire angle-.44rad exceeds the native tire limit.

New immutable public observation provenance records source pose/body, each sensor
source stamp, now/control clocks, nominal per-channel delays and serialized history.
Snapshot roundtrip and epoch/history mutation are covered by a regression. Stop tire
error diagnostics now compare physical tire to physical tire, independently of desired
steering. Complete-rest candidate feasibility now uses the shared body model instead
of the remaining wire=net reachability shortcut. Terminal fixed-zero speed also fixes
lateral velocity/yaw rate to zero in the QP. Integration count uses the common helper
for fixed5ms proof mapping; general configured-substep consistency still to audit.

No margin, tolerance, retry allowance, solver budget or production objective was relaxed.
M3 regressions and the remaining M1-M6 acceptance work remain open; no completion claim.

## Rest-mode comparison and proof repair

Buildr6:26packages pass (4min51s). Package r4:2387records/5failure records:
2related free-Stop cases plus1old seven-state comment assertion. All earlier
retained, cross-peer and false-zero-endpoint regressions now pass.

Frozen current-world free-Stop:
`output/20260910-nine-state-mode-comparison-r1/snapshot/000000000009-52db8fa2e4c33e02-cruise-side-neutral-initial-nine-state-rest-mode/snapshot.yaml`,
fingerprint5970523660795198978. Final rest-mode tangent's velocity A/B/offset all0;
QP speed0 but physical1.61453m/s. Direct relinearization and replay-state
relinearization both certify after1correction under the same hard scene. Four
independent held-speed→brake laws atstages12..15 also pass the same nonlinear
wall/peer/terminal proof. No infeasibility claim. Logs `/tmp/mpcc-nine-state-mode-comparison-r1.log`.

Producer correction: seal `terminal_body_rest_required` in full execution artifacts,
check exact u/vy/yaw-rate rest in the shared physical adapter, and include this
failure in the existing physical-proof SQP correction loop. Its maximum3,
all solver settings and physical tolerances are unchanged. Nonlinear max-braking
oracles must also preserve the sealed wire law. A new regression refuses a
QP zero endpoint with physically moving input replay.

Remaining fixture snapshot367272d907e582e9 is preserved at `output/20260910-nine-state-rest-endpoint-r1/snapshot.yaml`.
Native replay ends u1.6440739894e-8,vy7.9191343076e-7,r6.4552875201e-6 with terminal
positive wire1.7609098316e-6. This is not the nominal rest mode. The free-Stop
producer now declares its model-derived terminal wire<=0 condition before solver
insetting. It does not round/clamp a solved command or waive exact rest.
Build/test after that correction is still pending.

## Frozen candidate verification

Buildr8 passes26packages; package r6 passes2387records with0errors/failures/skips.
Repository applicable pre-commit hooks pass. Native/public comparisonr3 uses
nativer2,8212values maximum4.4964e-15;1s holdout position MAE0.193661302m,
maximum0.675902423m. `evidence.json` binds sources, patch and preserved logs.
Earlier pending build/test statements above are historical. Fresh runtime follows.

## First runtime and map-support correction

226f93e0 single-r1 fails at2804; see `first-single-failure.md`. The same-input
replay localizes missing map support; actual map extension certifies the Stop.
Buildr9 passes26packages in4min53s; package r7 passes2388records with0errors,
0failures and0skips. Fresh repaired runtime remains pending.

## Current runtime and dev2 audit

HEAD b0478348single-r2finishes6laps241.24130249023438s/penalty0. Dev2-r1rejects
at D1decision938. [First dev2 failure](first-dev2-failure.md) records the exact
937/938replay, all rejected backend/formulation comparisons, the historical-vs-live
Stop comparison coverage defect and positive terminal-target witnesses.
No diagnostic backend, target or input sequence is production authority.
The live complete-rest producer retains the source objective; only the historical
maximum-braking builder owns zero-cost feasibility. Earlier broad free-Stop
wording above does not establish otherwise. Same-HEAD M3–M6 remains open.

Canonical current-world complete-rest audit repair: buildr11passes26packages;
package r8passes2388records/0errors/failures/skips. Rear/side-peer live/audit
identities and proofs agree. The canonical CLI reproduces938candidate
334c7985b802506b and its expected dynamic QP rejection. Diagnostic-only code.

## Terminal-time structural candidate

See `stop-terminal-time-design.md` and `terminal-time-audit-evidence.json`. Public
peer and coherent-tangent alternatives are not promoted. Native0.8s complete-rest
feasibility certifies938. Implemented explicit native/max-support terminal-clock
population, zero-cost Stop objective, retimed all peers and distinct clock IDs.
Buildr13passes26packages; testsr9pass2389records; canonical938replay accepted.
Fresh dev2and M4–M6remain open.

746ec926fresh dev2-r2fails first at D2decision991/world32f29644e23b6e68.
[Current failure](first-dev2-r2-failure.md) records exact990/991replay and fresh
Stop QPs whose exact dynamic proof rejects. Common-primal SQP comparison is next;
all runtime is torn down and user artifacts restored. No completion claim.


## Causal peer-prediction candidate

D2r2source388's own acceleration detects its approaching conflict at2.0374s,
where the CV proof passed. Frozen paired CA1holdout and exact new-native replay
are bound in `dev2-r2-diagnostics-evidence.json`. See `peer-prediction-design.md`.
The shared finite-acceleration prediction, model identity, dense sweep and
bounded individual callback timing are implemented. Buildr14passes26packages,
testsr10passes2392records; source388oldCV/990/991replays retain their outcomes.
Fresh runtime remains required. Do not mark M4–M6 complete.

## Certified Stop publication repair

2b0293d1dev2-r3fails D2decision957. [Failure and timing](first-dev2-r3-failure.md)
localize the lost certified terminal sequence. The selected feedback Stop now
materializes that exact sequence and revalidates before execution; stateless
sources are recorded as published Bundles, including Stop. Full source execution
is recorded only when the command actually owns that source trajectory.
Buildr15passes26packages; testsr11passes2393records; production-API replay of
the reconstructed954alternate preserves its first command exactly and certifies
the materialized full rest suffix. Fresh runtime and M4–M6 remain open.


## Temporal complete-rest publication

d7fffe15dev2-r4executes materialized Stop322at846..852, then fails853when
control-origin u/vy/r are0and the original cursor expires. Current Stop proof
passes, but its temporal zero-distance interval cannot be represented by the
old positive-speed/strict-distance adapter. See `rest-publication-design.md`.
Constrained temporal rest representation passes buildr16:26packages,
testsr12:2394records, and exact852/853production replay. Native zero rest and
all current wall/peer checks remain. Fresh runtime/M4–M6are still open.

2026-09-10: f1f213fcdev2-r5reaches ShiftOut/Pass and canonical Rejoin after
relative-progress watchdog abort. Ego moves46.198m; faster peer84.979m, so this
is not physical standstill. The old monitor's tactical-phase stop is classified
separately from final Recovery override. Sixlaps remain incomplete and7adjacent
callback-overrun pairs reject timing. Observation-only nested timing passes
buildr17/testsr13:26packages/2394records. Next: bounded ShiftOut timing capture,
then causal repair and M4–M6. See timing-attribution-design.md in current steering.

2026-09-10:08322c3bdev2-r6startup stalls beforeReady; preserved/inconclusive.
Dev2-r7bounded timing shows D2decision1154marker16.559ms/924points and an adjacent
1155overrun with Recovery16.816ms. Negative-side ShiftOut→Pass→Return→Idle is
observed, but no integrated acceptance. Native identical-point comparison accepts
SPHERE_LIST batching (native prototype and direct production builder comparisons retained). Producer now
replaces complete display and clears legacy IDs; no control/proof/rate changes.
Buildr19andtestsr15pass. Fresh dev2-r8 timing and remaining M4–M6 follow.

2026-09-10:3ab839fadev2-r8fails moving Emergency atD2decision2144 after
negative-side ShiftOut/Pass/Return/Idle. Marker cost0.092–0.329ms; raw adjacent
overruns0/0, but only a short failed run, source duplicates10/10unattributed.
Paired exact2143/2144replay shows predicted control-origin speed drives rejection,
not peer update alone. The newly published acceleration can cancel the assumed
braking prefix used to certify its initial state. Native current provenance
replays exactly; prospective−3prefix yields a feasibleStop. Existing full5sStop
SQP also certifies this world, so it is not physically impossible. Publication
input binding is reopened within M1–M3; see publication-prefix-design.md.
Next: complete candidate/packet-bound native proof, implement shared prospective
prefix binding across normal/Stop/artifact/async, regress/build/run and M4–M6.

2026-09-10: prospective publication candidate implemented under existing autonomous
M1–M6 authorization. Shared native prefix is bound to each actual float packet,
including Stop materialization and final common candidate conversion; Follow ego
origin is updated against the unchanged canonical peer. Buildr21:26packagespass;
testsr17:2401records,0errors/failures/skips. Earlier testsr16 source-layout failure
is preserved. Native r8 replay rejects accelerating command under braking prefix;
matching Stop passes materialization/join/packet and snapshot roundtrip at+0.909311m.
See publication-prefix-design.md and publication-prefix-evidence.json. Fresh
committed dev2-r9 next; no integrated or M4–M6 completion claimed yet.

2026-09-10:99fccb87dev2-r9rejected at initial rest on both cars. Causal
native comparison exposes QPx0equality copied into initial physical corridor;
new prospective prefix moves1–2mm and is rejected before wallproof. Producer
now uses sealed initial physical map support while keeping QPx0/futurebounds.
Buildr24:26packagespass;testsr18:2402records/62groups/0errors/failures/skips.
Nativebeforefails; originalTrack294sourcebuilders solve/prove for both cars.
CurrentD1source840solves/proves, currentD2source819solverrejects at4000iterations,
kept separately. Details/evidence in bootstrap-bound-design.md and JSON.
Freshcommitted dev2-r10next; M4–M6and full integrated acceptance remain open.

2026-09-10: edad4e32dev2-r10 rejected on D1moving Emergency1670. Bothcars
launch; D2negativeShiftOut reachesPass. Ordinary1667/1668 exactnativepair
reproduces terminaldynamic boundary; reboundbraking1668passes1.212136504m.
Actual1669/1670pair missing because stationaryCruise833consumed finalbucket.
D1callback1668=164.662143ms; packetstamp28.284999367 arrives between public
clock28.434999364/28.439999364. Backdatedhistory is a causal hypothesis;
actualphysicsapplicationepoch and heavycallbackproducer need further evidence.
Observation adds separate firstmovingfailure bucket and eachoverrun's joincosts.
Buildr25passes26packages; testsr19passes2402entries/62groups/0errors/fails/skips.
Freshdev2-r11 next; sameM4–M6 remainopen. See moving-failure-observation-design.md.

2026-09-10: ae2137c6dev2-r11 rejected D1movingEmergency2588. Firstmoving
recorder now preserves exact2587/2588/source2047. D2negativeShiftOut→Pass→Return
→Idle observed. Root: normalStop reference ends~0.102m beforecompleteRest;
InvalidLateralPolicy then masksascenterlinewallcollision. Explicitconstantlateral
tail boundedby sealedStopmap fixes exact2588;2587unchanged; fullrest/wall/peer/
productionwirepacket checks pass. Solved-onlydiagnostic and strictsamplerremain.
Buildr26passes buttestsr20oneexistingdiagnosticfailure reportedtwice; corrected
buildr27passes26packages,testsr21passes2404entries/62groups/0errors/fails/skips.
Oldr10D1decision1668peerrejection remains, includingproductionadapter rejection;
itsbackdatedpublication/largecallbackcause remainsopen. Freshcommitteddev2-r12
next; M4–M6and fullintegratedacceptance stillopen. See terminal-reference-design.md.

2026-09-10: 8bfdaa52 dev2-r12 fails D2 moving Emergency933 during certified
Stop. Exact932/933source424 native peer clearance +0.042140739→-0.000833389m.
Same-peer-generation, latest velocity still rises after firstbrake; application
phase unknown in that run. Existing probes reused read-only in separate
application-dev2-r1:627 exact applied-input joins, firstfailureD2 1294 is intent
mismatch after ShiftOut→FollowPrepare→Idle. All evidence retained separately.
Observed prescribed application improves148D2brake-crossing speedMAE0.068265→
0.024975m/s; it is unavailable as production input and no r12 phase is inferred.
Fix known backdating producer: serialize history uses fresh ROSclock immediately
after publish with causal decision lower bound; packetstamp/prospective proof
remain nominal. Before2regressionsfail; buildr28passes26packages/4min48s and
testsr22passes2406records/62groups/0errors/failures/skips/28.78s. Receiver timing
uncertainty remains open. Same1294 actualpublished778 completeStop rejectsCruise
identity but accepts upstreamShiftOut/+10.061819599m/345samples, materialization/
rejoin/productionadapter; observation only, explicitStop handoff repair next.
AandrightB/C/D/G also certify1294; leftarms and2fullrestclocksreject. No
integrated acceptance or M4–M6 closure. See publication-epoch-design.md/evidence.

2026-09-10: aec8d32e publication epoch producer committed. Separate explicit
publishedStop handoff now keeps actual source identity after tactical intent
change. New regression fails old producer; buildr29 passes26packages/4min14s.
Testsr23 newnativepasses but2source-structure references fail; correctedtestsr24
passes2407records/62groups/0errors/failures/skips/29.07s. All forbidden publish/
store checks and generic normal/Stop intent equality remain. Native toolr30
actual1294published778: requestedCruise, proofShiftOut, completeStop345samples,
clearance10.061819599m, materialization/join/adapter and serialized packetmatch.
R12negative933source424 stays peerreject(-0.005110710497m)/no bundle;932unchanged.
Fresh committeddev2-r13next, covering both publicationepoch and Stop handoff
changes. Receiver application model and all M4–M6 remain open; no failed run
is relabelledpass. See intent-stop-design.md and intent-stop-evidence.json.
