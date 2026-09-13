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

2026-09-13 force-r3 onb6c6d04f/controllerfe976b39/build121:19604force/3849rx/972apply, normal through1148 thenstationaryloss1149;120s cap/noStart/laps. R568fixed helper/protocol unchanged,25unique receiver captures/35aliases,16physicalmatches/9pre-Ready unmatched. uMAE0.011776→0.005892but unsupportedmax0.038209remains; IMU5better/3worse. R569past extrapolation rejected forvy/r and maximumvy/tire regressions. No observer/model/guard change. OriginalDLL/JSON restored, runtime stopped. Next bounded exact-common-epoch coupled measurement event observer; no new live without material production improvement or distinct missing evidence; allM4–M6 remain.

2026-09-13 R570–R575:21native event reconstructions/16physical matches, no zero/borrowed seed, exact global-frame body invariance; candidate rejected for lateral/r/tire regression. R572corrects initial tire toBefore Vehicle update,21public endpoint maxerror1.054e-8rad. Legacy reader exact/five negatives pass. R575corrects R540tireoutside12→0of30(no bound change), body24speed misses remain; five of11140R555next-Sleep targets corrected. R573private oracle conversion defect retained/superseded byR574:15cases actual delayed input max tireerror2.386e-8rad, body/contact/rest errors remain. No production/run/guard change. Next missing3Dcurrent force structure, no repeat unchanged observer or blind live; allM4–M6 remain.

2026-09-13 R576–R582:11140native current-contact derivative matches, cached ground axes/point velocity attribution; R578542anchors/14310original predictions exact,541cached-supported but isolated fixes not promoted. R579one seven-feature contact law uses old4200training only, reproduces42876prior outputs, rejected for oldD1/holdout. R58013048public records(noTFstatic); R581182IMU/454pose matches confirms IMUattitude observable, filtered odometry not directheave. R582181finite-pose gyro comparisons plus1unsupported: public gyro matches5msquaternion delta, not instantaneous Rigidbody omega. No publisher defect, new live, model/gate change or M4–M6acceptance. Next explicit measurement/body/contact semantics and causal raw/derived support; avoid more unchanged point-contact fits.

2026-09-13 post-motion stationary evidence: old stationary buckets miss final1012(single-r7)/1149(force-r3) after earlier779/787. Add one lifetime paired bucket with authenticated moving-send witness; original buckets/authority/model/guards unchanged. R583 fixture collision retained; R584old2fail/new2pass, source106, build122all26/package109all2615, R586all3original wall/deadline identities retained. Next committed bounded single-r8; Stop/rest/restart and allM4–M6 open.

2026-09-13 single-r8/ec8e822b: bounded post-motion recorder captures final1052 due/active and authentic prior moving normal1001; no normal after1051.4229callbacks max19.291101ms,508normal sends, no deadline violation, no restart/Start/laps. R588 public max1.148950815m/s. R589 exact RestExpired: deadline14.314999826 beyond rest14.299999690; default CurrentCheck is unexecuted. R590 earlier430 numerical/622 physical A failures, target-free B/C/D/G unavailable; not later fresh-source root. Next bounded upstream failure observation after actual final loss, with both native initializer owners. Source/model repair and allM4–M6 open; runtime stopped/protected restored.

2026-09-13 post-loss source observation: actual queued final pair anchors immutable source provenance; original source buckets retained plus fixed per-native-initializer captures of the first subsequent failures. Optional received-body payload only, Snapshot layout/authority/model/fingerprints unchanged. R591/R592 fixture errors retained, R593old3fail/new3pass; source106/build123all26/package110all2618; R594all4original wall/deadline/RestExpired identities preserved. Next committed diagnostic single-r9; source root and M4–M6 open.

2026-09-13 single-r9/b368ecab, R595–R606: four post-loss failures1080/1081/1986/2703 captured, both owners verified.1160followed177normal sends; last1342/source1374, actual1343new1402current wall/old1374RestExpired.804normal,4275callbacks max21.185262ms,zero deadline violations,public max1.05967m/s,noStart/laps/restart. Store progresses to2700/~40s despite no sends;369of370later scheduled proofs wall-reject, so source shortage alone is not the root. Exact1081affine infeasible,1986affine feasible/cold fullsource certifies; independent native forward witnesses exist. R604same1986fresh counterfactual still scheduled wall-reject30.209999331. R600/R602/R605failed diagnostic setups retained; R606same programme native endpoint witnesses. Full observation/body/contact and applied/current proof repair plus M4–M6remain. Runtime stopped/protected restored, no production/model/gate change.

2026-09-13R607–R624: Paired last-stage branches and ordered3phase model remain rejected. Full ordered1641histories certify offline. Chosen fixed numerical velocity frame retains original arbitrary input population, no FIFO/model/profile/safety change. R616full1986wall/rest passes; R617256native traces/768000endpoint checks all enclosed, isolated prediction7.175861ms/75steps/4partitions. R619/R622five historical wall/deadline/rest negatives preserved using original source/domain evidence, changed wall rejection epochs recorded. R623original firstwallcell regression fails old producer; R624typed per-call production frame and cache-frame matching pass53native tests. Full new build/package/replays/live/local commit and M4–M6pending. See velocity-correlation-design.md/evidence.json.

2026-09-13R625–R629: Package111exposes four original moving-Stop positives lost by one coordinate frame; retain all expectations. Paired complete physical/actuation enclosures restore them, R625exposes two exact prefix-parity consumers. One shared short/full population fixes both: R626215native tests, build12526packages, package1122621records, source112pass. R627full original1986world certifies through rest with768000native containment comparisons/zero violations; isolated prediction14.047511ms is not live timing. R628/R629five historical wall/deadline/rest negatives preserved with exact identities. Numeric producer only; native physics/inputs/constraints/timing/normal authority unchanged. Review/seal/local commit then bounded single-r10; M4–M6 remains open.

2026-09-13f2a89c14single-r10/R631–R643:551normal sends,1142callbacks max20.404766ms,zero actual window violations; Ready only.999current physical accepts then actual before guard fails;1016programme absent,1020source wall rejects,1021normal resumes. Publicmax0.9889339209/after1000max0.1938343942. R633exact current14.2ms; R634actuation9.84ms dominated by missing physical-cache reuse. R635coordinate-conjugated parent maps reduce14.4to11.7ms, preserving full proof/late guard. R637old reference-center reuse regression fails; R638216native tests, build12626/package1132622/source112pass. R639positive/768000native checks and R64011.726–11.835mscurrent result pass; R641–R643old/same-run negatives preserved. Model/inputs/safety/clocks/authority unchanged; next local commit and single-r11, allM4–M6open.

2026-09-13fdee0b8fsingle-r11/R645–R651:120sReady-only,4255callbacks max19.919364ms,zero before-guard/actual-window violations;509normal sends end1047/source651.1048RestExpired exact3replays, no fresh certified Store after651. Both original source804/805outer wall stage71fail; coldA sameclass, B/C/D/Gtarget-free unavailable. R650quarter-drive certificates are only0.0034mps/6mm, not useful restart. R651all4native preparation/launch candidates fail. No source/model/profile/safety/supervisor change. Next exact cell/body geometry and target-free nonlinear source comparison; RecoveryReadyeligibility and measurement/contact separate, allM4–M6open. See transport-single-r11-audit.md/evidence.json.

2026-09-13R653–R656:source805front-left cell476325; native3seed comparison no useful restart (one4.496cm certificate, others rejected), Unknown. Separate prepared simulation Ready Recovery session and canonical Rejoin handoff implemented; no direct normal rejoin feedback. Native228/source115/build127/package114pass. Fixed-source single-r12and allM4–M6pending; source wall/model error not repaired. See recovery-session-rejoin-design.md/evidence.json.

2026-09-13R659–R669:single-r12failed with confirmation-cancellation loop and decision865deadline. Preserve accepted confirmation via existing reason, retire static/V2X/unchanged-budget force-motion bypasses, wire simulation declared membership without treating missing data as clear. R664oldfail/R665150pass; R667148Recovery+825V2X/source117; build128/package115; actual launch5conditions pass. Exact865replay separately recorded. Next fixed-source single-r13; all integrated M4–M6remain. See recovery-confirmation-clearance-design.md/evidence.json.

2026-09-13R671/R672:single-r13 confirms confirmation ownership and strict no-override SafeStop, but actual participant XML lacked recovery_vehicle_count (0). R674all5actual-entry cases fail; R675all5pass after XMLbinding; build129/package116pass. Controller07897bed unchanged. Next source-fixed single-r14anddev2-r80; all integratedM4–M6remain. See recovery-launch-entry-design.md/evidence.json.

2026-09-13R677–R686:single-r14bounds timing but Rejoin source supply deadlocks on publication override; dev2-r80propagates correct topology but actual D2send593crosses25mswindow. R683old source fail; R685151Core/118source/build130/package117pass with separate planning eligibility and unchanged final authority. R686exact before/actual-after replay preserves failed send. Next fixed-source single-r15for supply; separate publication-transaction structural audit and allM4–M6remain. Diagnostic setup failuresR682/R684preserved. See recovery-rejoin-supply-design/evidence/validation.

2026-09-13R688–R697:single-r15startup clock stalled/no usable bag; single-r16candidate supply resumes but acceptedRejoin952/source528is overridden by wrong source/current-ID equality. R694actual producer reproduces; R695compares immutable handoff alternatives. R696fixture coverage failure retained; R697explicit original/physical/domain routes243native and source118/build131/package118pass. Retire scalar identity heuristic; authenticate exact current opaque dispatcher without changing original IDs or publisher endpoints. Next local commit/single-r17; r80actual deadline,source walls/physical model and allM4–M6remain. R691corrects out-of-clock-range interpretation. See recovery-rejoin-handoff-design/validation.

2026-09-13R699–R701:fixedb3613667single-r17validates actual certifiedRejoin and two Normal/Cruise returns;856normal sends,4296callbacks(max26.410175),pre-refusal863/noactualpostviolation. Later Cruise pulse/brake and late reserved results lead to renewedRecovery1773and unknown-directionSafeStop; noStart/laps. Next bounded capture of full post-Rejoin reserved worker request/eachproof/timing, then same-world native programme/scheduling comparison. Runtime stopped/protectedfilesrestored; all remaining M4–M6open. See post-rejoin-source-audit/evidence.

2026-09-13R703/R704:bounded post-Rejoin reserved-Cruise diagnostic captures authenticated original Rejoin command/actualsend, allmax4attempted sources (including failures), separate worldfiles and evaluation timings. R704193native/source118/build132/package119pass; normal control/model/clock/reservation/safetyunchanged. Next local commit and single-r18capture, exact native replay and programme/scheduling comparison. See post-rejoin-observation-design/validation.

2026-09-13R706–R717:9c522a95single-r18no certifiedRejoin/Start/laps,357normal,max19.482284ms,noactualdeadlinefailure; post-Rejoin recorder not eligible. Source537wall144/cell471815reproduced. R715rightpreparation witnesses pass; R716freeQPoldoracle positive exceeds semantic2mpscap and is rejected as normal. R717three structural arms show coherent native post-refinement tangent+originalvelocityguard passes one existingcorrection/fullphysicalproof; originalaffinecorrection hitswall529. No production promotion. R713compile/R714input-envelope setupfailures preserved; R710v2loaderinapplicability not infeasibility. Next audit applicability regression, semantic bound provenance/coherent correction, additional worlds/build/package/live; all M4–M6 remain. See rejoin-wall-source-audit/evidence.

2026-09-13R719–R721:repair target-freeRejoin audit terminal applicability, oldRejoin-only failure/new50architecture/source118/build133all26/package120all2640pass. Actual new archive preserves source537outerwall144negative and no-braking rejection. Production control unchanged. Next semantic bound/coherent nonlinear producer implementation and additional frozen-world/live validation; no integrated acceptance claimed. See rejoin-audit-validation.

2026-09-13R723–R731:source native state proof and coherent post-refinement correction implemented. Original oracle speed-cap omission reproduced R727; native179/source118/build134/package121pass. R724free-seed diagnostic positive and original/drive4negatives; R729same four raw candidates and R730three extra worlds retain full semantic/wall/solver boundaries. R723/R726/source-text setup failures retained. Preparation population, current/retained consumers, live single-r19 and M4–M6 remain; see nonlinear-source-state-design/validation.

2026-09-13R732–R738:single-r19/40d1664b2325normal,2Rejoin completions,max22.09ms,noactualdeadlinefailure; pre-send863/1194remain. Game-wideStart observed; vehicleStart/laps absent. Eligible post-Rejoin recorder validated, fulljob1219input/attempt and domain replay fingerprints match. R736full-horizon appliedwall rejection precedes final3packet programme; source speedcap is not cause. R737span-first same certificate/programme/domain reduces~83to43ms offline. Additional frozen contexts and negative contracts before producer patch; no production ordering change yet. See reserved-program-order-audit/evidence.

2026-09-13R739–R743:five frozen order holdouts preserve admission classes and zero-prior fingerprints. R740old2proof-attempt failure; producer now tries distinct reserved span first and retains original full/short/ref alternatives once each. Shared count uses actual selected command cursor. R741native166/source118/build135all26/package122pass; R742six actual archive inputs preserve classes/exact accepted programme/domain identities. Next fixed-source single-r20; all physical/current/clock and M4–M6 obligations remain. See reserved-program-order-design/validation.

2026-09-13R744–R746:single-r20/7468cf46normal534,oneauthenticatedRejoin970,noRejoinComplete/vehicleStart/laps; game-wideStartpresent. Max17.98ms/noactualdeadlinefailure/pre-send1111refusal.13reservation invalids=9late(1reserved)+4timely;577/780capturedgenerationinactive. Missing Stop/wall profile first observed1242/20.01sec, exact later producer input missing; no clearance waiver. Complete earlier548world available for bounded preparation/free-QP holdout. See reserved-order-live-audit/evidence; all M4–M6 still open.

2026-09-13R747–R749:independentRejoin548, same fixed raw4preparation candidates=3physical positives. Paired free-QP study accepts right-drive2(81.08ms) and preserves originalwall720/right-drive4wall957.548source-only means original live719warmiterate unavailable; raw cold-controls progress rejection reported separately. Next declare bounded Rejoin initial-tangent population and snapshot compatibility, then focused tests/source replay/fullbuild/package/live. No production preparation yet; see rejoin-preparation-holdout-evidence.

2026-09-13R750–R756: Rejoin numerical preparation added to existing two-owner population;
free objectives/boxes and all native wall/dynamic/Stop/current/clock guards unchanged.
Old2regressions fail, offset-sign fixture errors retained and corrected; R753native208/
source118/build136all26/package123pass. Actual repository537/548 preparation passes;
430/622originals and527 original solver failure retained,527preparation wall-rejects.
R754recursive-policy inapplicability and R755optional capture metadata assertion retained;
R756 validates existing per-case evidence. Next localcommit/original-DLL single-r21;
actual source admission, callback/final-send windows and remaining M4–M6 stay open.
See receiver-input-enclosure/rejoin-preparation-design.md and validation JSON.

2026-09-13R757–R759:single-r21/6f42d134normal408/max19.53ms/noactualdeadlinefailures.
Recovery913thenunknown-directionSafeStop; no Rejoin initialization eligibility or
laps. Seven invalid reservations=4latezero-prior/3timelyreserved,914inactive/prefix
mismatch follows override. Capture exact Recovery candidate population/world before
any producer fix; source physical positives do not authorize unknown geometry or
detector threshold changes. All M4–M6 remain. See rejoin-preparation-live-audit/evidence.

2026-09-13R760/R761: one first failed CheckClearance direction records actual four-site
physical candidate inputs/results and course previews, original immutable map and
selection permissions. V2X summary only. No candidate/authority/guard change. Native245/
source118/build137all26/package124pass; saved physical/course positives/negatives,
overflow and duplicate checks pass. Next localcommit/original-DLL single-r22 and native
per-trial replay; actual eligibility/timing and all remaining M4–M6 remain open.
See recovery-direction-observation-design/validation and internal schema.

2026-09-13R762–R765:single-r22/1b478b27normal1133/actualRejoin>=3/onecompletion,
max24.70ms/noactuallate;1489pre-refused. Preparation sourcecertifications separate
from individual published-policy mapping. Capture1921complete/4trials, nativeR764
allmatch; three8mreverse wall rejects at6/3.55/7m. Earlier moving rear hazard remains
uncaptured; retry increased4to8. Compare base4and bounded piecewise8on same world,
preserve original hard oracle and no raw normal promotion. AllM4–M6stillopen.
See recovery-direction-live-audit/evidence.

2026-09-13R766–R771: Recoveryの新規後退候補を補助MPCの有効・無効から分離し、既存設定の操舵角を探索する。
保存1921では元の8m候補3件が壁拒否する一方、−0.05/−0.10radは元の壁・コース条件を通る。
R768旧選択失敗→R769native249/source118合格、R770完全地図で元3件の拒否を保持し
−0.05radを選択。距離・設定・壁・V2X・停止・通常権限・期限は維持。build138全26、
package125合格。次は固定commitのsingle-r23で実選択・再検証・時刻と周回を確認する。
全体は未完。先行する後退中の障害停止、車両Start・周回・M4–M6は引き続き未達。
See receiver-input-enclosure/recovery-steering-population-design.md and validation.json.

2026-09-13R772–R775: 226e4f84のsingle-r23は通常522送信、4260callback最大23.50ms、期限外送信ゼロ。
認証済みRejoin送信4件以上を確認したが、復帰完了・車両Start・周回は未達。
Recoveryの実選択12件はすべて前進で、小操舵の後退実行は未検証。方向不明の保存もなし。
後半は停止軌道のコース形状を生成できない。保存したRejoin537/538・656/743を再生し、
現在接触・軌道の壁拒否・後段の入力欠落を分離する。全安全条件を維持しM4–M6を継続。
See receiver-input-enclosure/recovery-steering-live-audit.md and evidence.json.

2026-09-13R776–R783: R776–R782でsingle-r23のRejoin入力を監査。537は現在姿勢が無接触で、3.615秒先の
壁接触を再現した。補正の継続条件へ完全な壁検証を加える診断では、537/656が元の
最大3回内に状態・壁・他車・停止継続を満たす。壁の近似制約を再生成する別方式は
537で拒否、656で合格。本番は未変更。次は現在接触などの負例と追加入力を検証し、
元の上限内の最小修正を実装・build・実走へ進める。全体・M4–M6は未完。
See receiver-input-enclosure/rejoin-full-wall-feedback-audit.md and evidence.json.

2026-09-13 R784–R788: 完全な壁検証で将来の軌道接触を検出し、元の最大3回内で数値補正を続ける修正を実装。
現在接触・既通過区間の異常は追加補正の対象にせず、最終の壁・他車・停止認証を維持。
R785旧版の実入力テスト失敗→R786native162/source118合格。R787本番ビルドで
Rejoin537/656の認証が成立し、追加3場面の元の拒否を保持。build139全26package、
package126は2655tests合格。次は固定commitのsingle-r24で計算・送信時刻とRejoinを確認。
全体・車両Start・周回・M4–M6は未完。
See receiver-input-enclosure/rejoin-full-wall-feedback-design.md and validation.json.

2026-09-13 R789–R792: 1648f338のsingle-r24は通常552送信、4267callback最大24.30ms、期限外送信ゼロ。
壁補正1回を経て認証されたsource785から3送信を確認。Rejoin送信2件以上、復帰完了なし。
ゲームStart成立、車両Start・周回は未達。後半は現在の物理的な横幅区間が採用されず、
停止コース形状が欠落する。現ログはpreferred containmentと実区間を欠くため、
次は元の入力・地図・区間結果を一度だけ保存して再生し、生成元の原因を確定する。
全安全条件を維持。小操舵の後退実行と全体M4–M6は未完。
See receiver-input-enclosure/full-wall-feedback-live-audit.md and evidence.json.

2026-09-13 R793–R794: 現在の壁区間が拒否される最初の実入力を、通常走行の認証済み送信後に一度だけ保存する
観測を追加。地図・姿勢・元の区間計算・preferred containmentを保持し、走行判断は維持。
R793native220/source118、build140全26、package127全2658tests合格。
次は固定commitのsingle-r25で実入力を取得し、元の地図・安全条件のまま一致再生する。
壁区間の原因は未確定。車両Start・周回・全体M4–M6は未完。
See receiver-input-enclosure/current-wall-interval-observation-design.md and validation.json.

2026-09-13 R795–R798: 38a562eeのsingle-r25で車両Startを初確認。通常3321送信、実期限外送信ゼロ。
4573callback中3646の1件が26.73ms（現在検証24.63ms）。その送信は元の期限内で、
送信前拒否19件からの通常送信はない。壁区間拒否・観測・Recovery操縦・Rejoinは未発生。
観測追加の効果とは断定しない。120秒内の周回は未確認。次は同じ制御sourceで
6周・600sim秒、660host秒上限のsingle-r26へ進み、全コースと時間・結果を検証する。
callback超過・未再現の壁区間原因・多車両を含むM4–M6は未完。
See receiver-input-enclosure/current-wall-interval-live-audit.md and evidence.json.

2026-09-13 R799–R806: single-r26は実送信2180の期限超過で中止。721正常送信とは別に、期限外の通常指令1件と
直後のfailsafeを保存。元の指令・証明・履歴がR805で一致し、送信前許可→送信後拒否を再現。
callback3.46msでもROS時刻が25ms進む。直前の記録側clock受信間隔は297msで、原因の
描画・GC・DDS等は未確定。単純な5ms余裕や期限拡大は採用しない。次は元DLL・物理・
制御を保つ描画なしsingle-r27診断で時計と送信を比較。r25車両Startは別runの確認済み証拠。
全コース・周回・統合受入れとM4–M6は未完。
See receiver-input-enclosure/full-course-publication-timing-audit.md and evidence.json.

2026-09-13 R807–R811: single-r27の描画なし診断は通常738送信、4281callback最大24.392ms、実期限外送信ゼロ。
時計の記録側受信間隔中央値5.001ms・最大14.327ms（標準描画r26は中央値0.574ms・
最大296.983ms）。環境差への支持はあるが、経路・場面が異なり描画の原因確定ではない。
Recoveryは後退25/前進3選択、小操舵−0.05radの指令と実負速度を確認。Rejoin送信2回、
復帰完了・車両Start・周回はこのrunでは未達。壁区間拒否は初の通常移動記録より前に
現れ、既存の観測範囲から外れていた。last_physicalは過去値の保持で、連続拒否を意味しない。
次は先行通常移動を必須としない初回実クエリ観測へ置換し、原地図・元の判定を一致再生する。
標準描画の送信期限問題と全コース・M4–M6は未完。

2026-09-13 R812–R814: 初回の壁区間拒否を通常移動前も保存する観測へ修正。先行通常記録は任意の実証拠とし、
元の地図・クエリ・判定・安全条件を維持。R812旧版1失敗を再現、R813native34/source118、
build141全26package、package128全2660tests合格。次は標準描画のsingle-r28で初回実入力を
取得して一致再生する。壁区間の原因・標準描画の期限問題・全コース/M4–M6は未完。
See receiver-input-enclosure/first-wall-interval-design.md and validation.json.

2026-09-13 R815–R820: 7f263cc2の標準描画single-r28は通常675送信、Rejoin送信3件・復帰完了1回、実期限外送信ゼロ。
4271callback中495の1件25.218ms（初期化14.746ms/Recovery6.402ms）、連続超過なし。
2771は送信前拒否。ゲームStart成立、車両Start・周回は未達。壁区間拒否は発生せず観測なし。
後半のRecovery2906をR818で完全再生。前進3候補は全て壁またはコース条件で拒否され、
「前進優先」の分岐が後退評価まで省く。これはordering-onlyという宣言と不整合。
R819は同じ地図・姿勢で既存後退4m/+0.15rad、8m/+0.10radの物理・コース検証に合格。
未評価の元の後退距離は未保存なので、これらは明示した距離での診断であり実行許可ではない。
次は優先指定が候補を消さない選択処理の回帰・修正・検証。先行する通常停止の原因、
標準描画の期限問題・全コース/M4–M6は未完。

2026-09-13 R821–R824: Recoveryの前進優先を候補順序として実装し、不合格なら残りの既存候補へ進むよう修正。
元の前進評価と接触・段階的・後退の分岐内容を保持。同一クエリ内の同じ前進評価だけ
結果を再利用する。R821旧版2失敗→R822native73、最終source118、R823全地図の比較、
build142全26/package129全2664tests合格。次は固定commitのsingle-r29で実行・時間・復帰を確認。
未評価だった元の後退距離の完全再生、先行通常停止・期限問題・全コース/M4–M6は未完。

2026-09-13 R825–R829: 7ecd0b36のsingle-r29で車両Start、Rejoin送信5件・復帰完了1回、通常2741送信を確認。
実期限外送信ゼロ、送信前拒否22件からの通常送信なし。4500callback中2件超過、連続なし。
2267は28.585ms（normal join16.687/Recovery10.407ms）で送信前拒否、2286は26.109ms
（Recovery21.738ms、全体CPU14.764ms）。後者の処理内/off-CPU詳細は未確定。
Recoveryの実選択は前進5件だけで、修正分岐の後退実行は未観測。Startを修正効果と断定しない。
壁区間拒否・方向不明の新記録はなし。周回・全体M4–M6は未完。
次は同じ制御コードの全コースsingle-r30（6周/600sim秒、660host秒上限）で未到達範囲を確認。
過去r26/r80の実送信期限問題は未修復で、同じ元のガードと失敗時の中止を維持する。

2026-09-13 R830–R837: single-r30は実送信8440の期限超過で中止。正常送信2420件とは別に失敗した通常送信1件を保持。
元の入力・履歴・証明経路をR836で一致再生し、送信前許可→raw診断終了時点/送信後拒否を確認。
8784callback中644の1件25.030ms、連続超過なし。車両Start/Rejoin復帰1回、周回は未達。
壁区間1263は全地図・元クエリで完全一致。現在は非接触だが、1mm上側の安全余裕込み車体は
壁セル466554に接するため、既存境界ガードの拒否は正当。壁条件を緩めない。
原DLLの標準60FPS制限はbatchmode/nographicsでも解除される。r27は描画だけの比較ではなかった。
次は描画を保つ--target-fps 200だけのsingle-r31（120host秒）で制限の影響を切り分ける。
物理・時計刻み・制御・25ms期限とガードは維持。時計問題・全コース/M4–M6は未完。
