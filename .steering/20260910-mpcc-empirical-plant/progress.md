# Shared-model migration evidence (in progress)

Current controller baseline is `cbcda112`; the latest receiver-range and independent
application evidence is in [receiver-input-enclosure/results.md](receiver-input-enclosure/results.md).
Full M1–M6 remains open. The chronological checkpoints below are historical; their
build/test/run results must not be treated as acceptance of the latest implementation.

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

2026-09-10: e710056d dev2-r13 failed at D2decision1401 duringShiftOut, source828Stopwallreject; laterPass is afterfailure. Exactprior1400request unavailable,1399is historical. Adjacentcallbacks1400/1401=38.366/32.753ms. Stopmodelclosure atzeroerror exposes continuousfuturesteering versus a new heldpublication mismatch; continuouscounterfactual alone restoresall8samples but is never publishable. BoundedA/B/C/D comparison is inconclusive (15rejectedcandidates), fullA-Gtimeout retained. Stopproducer now uses float32heldangles, sealedincrementinputschema, unchangednativebody/rest/wall/peers/packetchecks. Buildr30/r31pass26packages; testsr25two oldassumption/classificationfailures corrected without removing negative checks; testsr26all2409records/62groupspass28.69s. Correctedhistorical1399Stop330samples/bundle/join and all8modelclosurespass. Exact1401stillwallreject; exact933peerreject-0.000891864256m/no bundle. No integrated acceptance claimed. Freshcommitteddev2-r14next, then remainingM4–M6. See stop-packet-schedule-design.md and stop-packet-schedule-evidence.json.

2026-09-10: 21b025f3dev2-r14 failed D2decision1112 unexpected Recovery while Stop604 remains certified. Actualstateless589publication retires frozenMission at1100; old earlyreplan wrongly reopens on absentgeometry. One shared applicability predicate now recognises frozen or actualpublished owner; ordering stays diagnostic for ownedencounters, legacy uncommitted Switch/Abort and physical supervisors remain. Nativebeforefails; buildr32=26packages/4m15s, testsr27=2410records/62groups/0errors/failures/skips/29.62s. Earlier589snapshot boundedA/B/C/D15candidates allsolverreject, Unknown; exact1112world absent, livepositive589publication is separateevidence. R14adjacentoverrunpairs0/0, stillfailed. Fixedcommitdev2-r15 next; all M4–M6 remainopen. See side-replan-ownership-design.md/evidence.json.

2026-09-10: cbcda112dev2-r15 fails beforeShiftOut at D2decision956/wp31/1.47m/s, Stop449peerreject. Current-controller application-r2 also fails1001/wp31/1.15m/s, Stop576; actualreceivertrace binds firstnegative503source11.729999737, afterpositive502application11.764874323, to−3application507at11.866269907. Native exact955/1000historicalinputs pass but sourceassociations are invalid for449/576. DirectfailureStop clearances−0.000425110/−0.000167764m; they remainrejected. Five conditionaldeliveryhypotheses show immediateStopaccepted inputs can fail with50mspositivehold; earliest994stillpasses150ms but995doesnot. No delay or margin adopted, no productionchange, no integratedacceptance. Explicit receiverinput scheduling is the next shared-model/proof slice; see receiver-schedule-design.md and receiver-schedule-evidence.json. All M4–M6 stayopen.

2026-09-11: shared body/tire arithmetic extraction preserves494,904scalar bits over37,956cases. Buildr33/testsr28 pass26packages/2,410records. Native enclosure r9 and four original source worlds r3 pass. Receiver scheduling/proof adoption and all integrated acceptance remain open; see receiver-input-enclosure/shared-kernel-design.md.

2026-09-11: shared applied-input library checkpoint after2fc31bf7. Same-epoch history2/3→3/3; future-packet and interior-gap regressions nowreject. Initial absolute-clock bridge failure and low-speed test expectation correction retained. Monotone tire wrapping repair passes160000native values; source library r2 passes4original worlds/374960values. Buildr37=26packages;testsr30=2417records/62groups/0errors/failures/skips. Five observational probes preserve3608original CIL methods; initial field alias and validation failures preserved. application-r4guard abort and r5actualD1failure966/438 are distinct. Predeclared100mssteering candidate false atstartup134.715637ms;1865complete intervals fit separatelyreported250ms. No runtime uncertainty authority or integrated acceptance. Next immutable common program/proof, all-response-rest Stop/clock/materialization/async/final packet, then M4–M6. See receiver-input-enclosure/applied-library-evidence.json.

2026-09-11: after95180a1c, complete common-program certificate connected to live requests/Stop materialization/async/final actual ROS floats. Frozen old two-stage serialization counterexample fails, current passes. First complete publication interval at rest is required. Buildr39=26packages/testsr31=2425records/62groups/0errors/failures/skips. Saved failed worlds966/956/1773 remain rejected without history repair; historical inputs are not true predecessors. Initial fixture mismatch archived. Next dev2-r16;25ms, actualStop/rest/restart andM4–M6 remain open. See receiver-input-enclosure/applied-integration-evidence.json.

2026-09-11: c86c7163dev2-r16fails D2moving798; actualStop124joinlost795on signed currentu=-.00230949 while futureu=0. Scalar observed source is unchanged; exact required applied proof nowowns its admissibility, future bounds unchanged. Native795replay/rest/peer+1.072677m accepted; missing proof/larger reverse reject. Rest map avoids Jacobian,256000nativevalues and fullenclosure/source suite pass. Buildr40=26/testsr32=2427records/62groups/noerrors/fails/skips. Failedfixture/compile/oldABIattempts retained. r16timing stillrejected; freshdev2-r17next, thenM4–M6.

2026-09-11:7b7fef37dev2-r17remains incomplete: certified rest persists without restart, with stationaryEmergency1020/source619 and repeated25ms overruns. Exact failure comes from convexifying actual -3/+1.329596 packets across missing near-zero values; same-world nominal A/B-left/C-left/D-left/rest solves succeed. Sign-group propagation nowaccepts exact1020certificate/materialization/join at unchanged profile/world/bounds. Buildr41=26/testsr33=2430records/62groups/zeroerrors-fails-skips; full continuous primitive and4savedworlds pass. Freshdev2-r18next, M4–M6stillopen.

2026-09-11:2c3d8d7cdev2-r18fails D1Stop249join839after alternate838consumed229.759ms(callback255.508604ms), causing259.999994msactualpublicationgap. Native839rejectsunchanged250msHistoryUnavailable; firstmovingEmergency843/.12m/s. D2later996/source497peerreject separate. Exactslow838/source241missing: boundedobserver nowcaptures firstalternateoverrun/request/timing beforefallback. Buildr42=26/testsr35=2430/62groups/zeroerrors-fails-skips; initialr34source-parser failure preserved and sameassertions repaired. Freshdiagnosticdev2-r19next; M4–M6open.

2026-09-11:4eb19997dev2-r19captured exact slow alternates D1854/source182 and D2836/source145. Native full prediction then reject + duplicate solved suffix costs135/89ms. Same necessary checks now stream; invalid samples stop prediction without any partial proof. Identical solved-suffix retry removed, distinct reference alternatives retained. Native r86~14/12ms with identical reason/time. Buildr43=26/testsr36=2432/62groups/zeroerrors-fails-skips. Positive r17certificate/join, signed r16join, negative r18gap and four-source4argAPI all pass. Same-world architecture arms recorded individually including complete-rest first failure/second success. r19first moving D1972wall and D21071peer input replays stay rejected. Freshdev2-r20 next; actual publication/25ms/restart and M4–M6 open.

2026-09-11:c87b9fa6dev2-r20fails D1964/2.29mps/WP33. Exactsource260wallreject12.25499975; previous963actualsource277separate. Diagnosticr89nominalgate-off changed selectedreference and was rejected as exactmethod; r90assertsfinalarm reason/time. Existing mean-value image discarded independently valid natural bound. Isolatedintersectionr91and conservativeoriginalwallfixture reproduce oldfailure/newpass. Productionsharedintersection buildr44=26/testsr37=2433/62groups/0errors-fails-skips;2017600continuous+956800hybrid values and4source suites pass. Exact964certificate/production/materialization/join passes; r19D1972wallalsoaccepted, D21071peerstillrejected. Freshdev2-r21next. Actualpublication/25ms/restart/M4–M6remainopen.

2026-09-11:4210fa89dev2-r21 fails D1953/1.88mps/WP32, exact source261wall11.739999760; previous952actualsource270 separate. Same-world A/B-left/C-left/D-left/G-left nominal pass, right reject, complete-restY passes. Prior retained publication programme alone is insufficient (derived steering step rejection; original suffix still wall-fails). The two lateral tracking terminal generators omit a constant serialized steering candidate. Isolated r100/r101 preserves actual first packet, uses existing next-interval minimum braking and same steering word; all original nominal/applied/wall/CA1peer/Follow/rest/authority/materialization/join gates pass on r21D1953,r20D1964,r19D21071. Production candidate bypasses complete solved Stops and records proof kind/cost.83focusedtests/buildr45=26/testsr38=2434/62groups/0errors-fails-skips; fresh native regressions pass, real r18history gap stays rejected. Freshdev2-r22next. Actualpublication/25ms/restart/M4–M6remainopen.

2026-09-11:5f673517dev2-r22 fails firstD11024/2.73mps/WP35. Capturedsource338lastreferencewall12.999999729; commonprogramme wallclear but peer d2reject13.559999729. Previousactual1023source365 is separate; its original remainingprogramme also peer-rejects after worldupdate. D21083failure is later, after D1Emergency. Sameworld persistentA nominal passes, B/C/D/G missingtarget inconclusive; complete-restY firstsolverreject thenaccepts. r106full original pose-box support separates despite inflated-rectangle overlap; r107one nearest-rectangle direction suffices. r108isolated fullproof/production/materialization/join accepts exactD1; D2rejectunchanged. Productiongeometry preserves allinputs/margins/CA1/state/world/identity; r46newtestlinkfailure corrected, buildr47=26/testsr39=2437/62groups/0errors-fails-skips and fresh native regressions pass. Freshdev2-r23next. FullM4–M6/25ms/recursiveStopremainopen.

2026-09-11:749a871cdev2-r23 fails firstD11018/source316/2.90mps/WP35; previousactual1017source342 is separate. Common immediate Stop peer rejects13.889999723; final reference rejects13.519999723. More geometry directions and original-bound braking/steering variants do not certify. Malformed sparse delayed r113is inconclusive; dense r114passes at .2/.4/.8s. Original source-horizon r115passes .275s/11positive packets and all gates, with cap reported only. Production now derives that programme and enforces exact original source speed cap through nominal/fullresponse/materialized successors and v2provenance. Rear-peer heuristic only ranks candidates. Buildr48missingtype definition fixed;r49warning fixed;r50passes. New test r40confused normal25ms authority with separate fullterminal proof; corrected with extra normal/source/fullproof and missing/truncated proof rejection checks, then buildr51=26/testsr41=2441/62groups/0errors-fails-skips. Automatic review initially rejected the test correction, accepted after code/proof-range evidence; no approval pending. Freshnative r116/r117 and committeddev2-r24 follow; M4–M6and actual25ms remainopen.

2026-09-11:e53e3791dev2-r24 fails firstD1947/source245/1.60mps/WP31; actualprevious946source249 is separate. Callback151ms; r116underlyingwall10.684999773 reproduced. All4existing terminal arms rejected; complete-restYandA/Bleft/Cleft/Dleft/Gleft nominal pass but fullapplied not established. R119prioractualprogramme stillwallreject10.969999773; finite64nativepaths clear. R120132posesinfirstboxclear. R121wholebox/wholecellsupport separates earlyrectanglecontact butfailslater11.054999773,min−.012405141m. Puregeometryorretentionaloneinsufficient. No productionchanges; see receiver-input-enclosure/r24-wall-audit.md. Continue numerical enclosure/root-cause and remainingM4–M6.

2026-09-11: Extra rigid-corner displacement enclosure repairs originalr24D1947wall
population, retaining every body bound/input/hybrid branch/margin. R119previous
programme alone fails; r120finite pose sample inconclusive; r121support alone
failsbefore rest; r122worldaxes worse; r123compilefail; r124singleextraAABB fails209;
r125rawtrigdifferences fails39; r126stable equivalent differences close all39
contacts while1744bodycomponents exact/111616nativecornervalues enclosed.
Production shares the native midpoint map/Jacobian and adds outward chord-sagitta
sweeps. Buildr52=26/5min;testsr42=2447/62groups/0errors-fails-skips/29.32s,including
2073600nativecorner/sweepvalues and captured/truecontact/invalidcontext tests.
Freshr127/r128 closes947fullproof-production-materialization-join10.725639ms,
rest11.174999773,2commands/210samples; past positives retained, realr18gap rejects5,
later r24D2990 and r22D21083peerreject remain. Registry188snapshots/584experiments,
305evidencefiles. Commit thenfreshdev2-r25; M4–M6 and live25ms remainopen.

2026-09-11:13be2d87dev2-r25 failsD1964/1.86mps/124.676ms. Original inspected212cursor is expired; actual263and current semantic observation remain separately available. R129reconstructs actual263normal wall failure and nominalStop->materializedStopfullwall failure11.499999763. R130compilefail preserved; r131128native paths clear/1505280values enclosed; r132per-branchcheckingalonefails57steps. R133/R1342/4/8forward-speed subdivisions all close wall, including joined new boxes; original inputs preserved. R135isolated2split fullnormal1attempt21.961207ms and publishedStop20.290395ms both accept. No production edits yet; r25-speed-partition-design is active. Registry addition pending with validation. M4–M6remainopen.

2026-09-11: Forward-speed partition production integrated: midpoint of existing
moving positive next-state interval, overlapping closed cuts, max6bins, all
original inputs/modes and physical/provenance/final gates retained. Buildr53=26/
26.2s,testsr43=2449/62groups/0errors-fails-skips/29.44s. R24historical geometry
frozenbyr136beforechange; current fullwall/native/rest checks retained. Newr25
fixture clears with all128nativepaths and >1millionbody/cornervalues enclosed.
Freshnative137-139:actual263normal18.28586ms,normalmaterializedjoin16.089412ms,
publishedStop16.308743ms accept; earlierpositives pass, signedr16pass, realr18gap
reject5. Later r24D2990nowpeer-clearsunder tighterbodybounds; r22D21083stillrejects.
Registry189snapshots/609experiments,315evidencefiles. Commit thenfreshdev2-r26;
actualpublicationclock and25ms/allM4–M6remainopen.

2026-09-11:963f9aa3dev2-r26failsD1928/1.20mps/WP30,actualprevious927/source143,
inspected131distinct;D2later996. Firstcallback150.382308ms. A/B/C/D/Gordinary
armsreject; secondcomplete-restYnominalaccepts31.9097ms, noappliedauthority.
R140previousprogrammealonefails; r141turningStop128directionsfails; r1422/4/8
speedbinsfail. R143previousprogramme/captured-firstconstantStopwhole-population
oblique separation passes. R144grid/midpointaxesfail; r145original-frameaxes
closeactual143normal anditsimmutableStopjoin, separate generatedStopstillfails.
Production wall support preserves original tube/inputs/footprint/margins/cells,
CA1peers/Follow/fullrest/identity. Buildr54=26/23.8s,testsr44=2451/62groups/zero
errors-fails-skips/29.74s;native146-148earlierpositive/realhistorygapregressions
pass. See receiver-input-enclosure/oriented-wall-evidence.json. Commit then
freshdev2-r27; actualpublicationclock/live25ms/allM4–M6remainopen.

2026-09-12 resumed f9aa990a: single-r5 D1Ready1122 exact current source/history/domain/lastactual reproduces physical acceptance and actual before-window rejection in R513. Bag317.259755ms receipt gap then40ms ROS within4.997012ms wall is not controller delivery attribution; old R39/R53 and rejected partitions are separate evidence. Run externally interrupted for analysis, no Start/laps/deadline result, protected artifacts restored. R514 cold source302accepts/373rejects/573different-boundary; target-free alternatives unavailable. R515–R517 native-seed comparisons find fully certified sources without changing hard constraints, but R518 global replacement loses existing586 and is unpromoted. Next bounded candidate architecture and independent clock repair; full M4–M6 remains incomplete. See receiver-input-enclosure/single-r5-current-audit.md.

2026-09-12 bounded native initializer population: R519 fixed reference-rest/original order certifies all6sources, including original586. R520 setupinclude failure retained; R521 old3fail/current31pass. Target-freeCruise reserves2sourceIDs before worker; explicit replay/fingerprint policy, independent solver/warm owners, first full certificate Store selection and exactly-one physical mailbox group result with newer-epoch revocation. Build120 all26/package107 all2610/source106 pass. R522 actualbuilder+built6fullsourcechains pass; R523 original1122 physics/late guard and R524 original1022 unsafewall unchanged. Only candidate count changes total bounded source work, original numerical/physical/time limits preserved. Next committed standard single-r6; all M4–M6 remains incomplete. See receiver-input-enclosure/native-seed-candidates-design.md.

2026-09-12:8d9810e0/build120/package107 standard single-r6 fails movingD1Ready968/source642/job961/index4 after967/index3. ExactR525 reproduces final context784 and wall968/12.629999725. R526 two upper-input native witnesses contact cell476325; no numerical wall relaxation. Original tube misses current yaw/u (R527), and R530 same-bag pre-failure scope confirms early component departures; R529 later post-Emergency comparisons excluded. R528 variable-source-rate3 source-certifies but8/16 still fail; no production promotion. All1166 callbacks below25ms/max21.074611, no Start/laps. Protected artifacts restored. Next bounded force/application observation; allM4–M6 remain open.

2026-09-12 force-single-r1 on e866ecf7/production8d9810e0/build120: instrumented Ready950/source632/job945/index2 wall failure12.519999728 exactR533, no Start/laps. R5311931force rows; R532807native samples isolate contact averaging as a contributor, with private oracle and3D limitations. R534497anchors reject all-grounded replacement on independent old single/D1. R535/R536165exact applications, no future transport bound. R53771zero/568one-contact oldsingle ticks; holdout35msall-airborne,60msat-most-one; private past forecast not uniformly better. No model/authority change. Protected JSON/DLL restored. Next exact sensor-epoch versus model attribution; allM4–M6 remain open.

2026-09-12 R538–R544: force-run source speed starts time-aligned/physically accurate; all30actual wires in original sign union, speed leaves tube in24samples. At source+140ms synchronized initialization barely changes0.02565mps error, future-contact-only oracle reduces to0.001659. R541 finds diagnostic kart-root/base_link mismatch; R542 rescores unchanged outputs, leaving speed/yaw/globalallground rejection intact. Corrected future-contact oracle improves position across new/old windows. Shared explicit-reference/domain reader added for diagnostics. R543 future-awake setup failure retained; R544453common anchors rejects all static-past forecasts on oldholdout1sposition/yaw. No production change. Next evolving contact-mode forecast comparison with public-input/promotion boundary; allM4–M6 open.

2026-09-13 R545–R549: fixed contact transitions give small nominal gains. R547 old training wire variation3.35e-7 and speed5.232–8.342 expose missing low-speed/braking calibration. R548speed/lateral law improves new/oldholdout but oldD1regresses. R549adds ONLY r1 8–10s400awakeDrive samples, labels training overlap/known regressions, freezes with/without-wire models before prospective force-r2. Exactprogramme145msspeed0.027035→0.002049/0.003193, yaw/oldD1still open. No production or gate change, no whole acceptance. New force observation will test fixed coefficients; allM4–M6 remain.

2026-09-13 resume: force-r2 completed D1Ready972/source643/job967/index2, noStart/laps. R550–R558 extract1810force/5036public rows,157exactapplications, reproduce original source/current wall rejection12.599999726 twice. R554frozen prospective42anchors improves1suMAE0.085013→0.041026,position0.057563→0.024681m without refit; all158baseline predictions exact. R557finds all27wires allowed but speed outside from source pose, where rawu is30msold. R556contact and component synchronization are independent contributors. R55511140steps retain 3D/contact limits. R558public common-past improves some source speeds but worsens current freshness/yaw and interpolates across launch; not promoted. Source8d/build120/package107 unchanged. ProtectedJSON/DLL restored, runtime stopped. Next actual received-history evidence, coherent causal observation/physical model repair and allM4–M6; no confirmation/push.

2026-09-13 received-body diagnostic implementation: existing256entry velocity/IMU/tire queues captured once per valid decision and separately owned by semantic solver source, scheduled source and current failure. Outside all interaction/model fingerprints, no selection/model/authority/guard change. R559missing include/R560fixture path failures retained, R561old3fail/new3pass. Build12126packages/package1082613records allpass; R562freshdriver reproduces force-r2 and single-r6 wall and single-r5 actual before-publication guard rejection with exact original identities. Next committed bounded single-r7; causal observer/contact repair and allM4–M6 remain open.

2026-09-13 single-r7 onfe976b39/build121: Readypublicu max1.03985, normal through1011/job994/source593/index14, final loss1012stationary/no exact final slot after earlier779.120s cap/noStart/laps. R56320unique receiver captures establish excluded newer public values;4239callbacks max19.778ms/all480send windows valid. R56435msvelocity/tire,50msIMU,control40Hz. R565frozen diagnostic interpolation with5negativecases; R5667original-history nominal comparisons show command-derived initial desired changes have no effect. OriginalDLL/JSON restored, runtime stopped. Next frozen force-r3physical pose-epoch comparison; observer/model repair and allM4–M6 remain.
