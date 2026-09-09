# Current status and remaining completion work

2026-09-09JST. Calibration baseline b530883997e80cd78aa7e70bcd53bf2b6bcf93fc;
the subsequent side-peer/Stop slice is fixed by its source/binary/run manifests.
The complete MPCC acceptance is NOT achieved. Preserve previous failed trials.
The user authorised autonomous execution through completion on2026-09-09.
Local commits bc01ff2f,ca38bf18,d54822c9preserve the previous completed work.
Commit5cfc50bcpreserves the coordinate repair and its evidence. Commit b5308839
preserves complete peer-body geometry and separate semantic physical initial state.
The new side-peer/Cartesian-plane/Stop numerical and candidate-scope repairs are
locally validated; coupled acceptance remains open.

Current next step: causal ego/clock/command audit of exact same376accepted934and
terminal-rejected935, plus derivedStop388provenance at final938. Observation-only
slice after9d76066c passes17native/104source/25package build/60CTestgroups2372records.
The prior Cruise accepted-input gap is repaired with one retained ordinary input
and atomic failure pairing. No control/proof limit change. World941A/B/C/D/Gand
support/full-rest comparisons reject, stillUnknown. Wire covariance uses metres
standarddeviation; a sqrt/unit fix was falsified.
Freshdev2:934acceptedterminal+0.017855530645m,935terminalpeer-0.000355659220m,
15msapart, both exact inputs replay withoutsolving. Peer-only substitution keeps
both outcomes, so ego/clock/command changes remain candidates.935independentStop
is stillaccepted. FinalmovingEmergency938/v1.34m/s: actual376, inspected388solver
source missing; priorordinaryAccepted937/376 cannot substitute for388. D1/D2
callbackmax21.762/19.321ms/0overruns,40Hzcommandreceipts/no gaps>50ms; source and
odometry/clock pauses remain. Runtime stopped/user JSON restored. See
[peer viability and paired observation](../20260909-mpcc-peer-viability/results.md).
Whole MPCC acceptance is NOT achieved.

Previous GNSS repair and941boundary:
The then-next step was bounded current-world peer/terminal viability audit of new
GNSS-repaired dev2 D1world941/actual370. Cumulative low-speed GNSS heading defect
is locally repaired:4of5newactual-nodecasesfail before, all5pass after;
25packages build, MPCC2371records pass, GNSS30records0failures/6cppcheckwrapper
skips (direct cppcheck clean). Same-recorded136/144GNSSfixes update yaw and rotate
lever arm, correcting up to0.460/0.496m versus old node output. No threshold change.
Single6laps254.349091s/penalty0, movingoverride0, callbackmax14.674ms/0overruns.
Dev2stillrejects D1decision941/v1.299626669m/s: exact terminal and final snapshots
both contain370and rejected Request. Zero-solve steering join passes, terminal
peer-0.001206851953m and independent Stop peer-0.000951979315m reject.
D1/D2callbackoverruns0/max20.730/23.434ms; source/observation delivery remains
separate unresolved evidence. All runtime stopped/user JSON restored.
See [exact join and GNSS results](../20260909-mpcc-exact-join-causality/results.md).
Full coupled/intents/dev3/dev4/gates/submission acceptance is not complete.

Previous observer baseline466bba5e and its945diagnosis:
The then-next step was causal audit of exact dev2 D1 world945 / actual and inspected361,
now recorded with its exact rejected Request. The first-event observer repair
passes16native/104source tests,25-package build and60CTestgroups/2371records.
Fixed dev2 captures the first moving Emergency945at1.5802064876844846m/s.
Zero-solve replay reproduces steering-unreachable, terminal peer-0.003647424709m
and independent Stop peer-0.001124094799m. Original361wall/dynamic pass;
current pose/speed/steering join differs. Both Domain callbackoverruns0,
commandreceipt40Hz/no gap>50ms, but source/observation delivery pauses remain.
Coupled race rejected. Later1137is downstream of first Emergency, before teardown.
Use exact publication/request clocks and same-run serialized commands to test
reachability ownership; compare A/B/C/D and full-rest candidate architecture on
this new world before another failure-family patch. No offline candidate is
promoted. All runtime stopped and user JSON restored. See
[final authority observation](../20260909-mpcc-final-authority-observation/results.md).

Previous e5b8c455 evidence and now-repaired recording gap:
The then-next step was to preserve the final-authority observation at dev2 D1 951 and
the actual Stop 379, distinct from first terminal world 949 / normal 365.
The metadata repair in [Stop proof provenance](../20260909-mpcc-stop-proof-provenance/results.md)
passes its failing native regression, 25-package build and 60 CTest groups /
2369 local records. It binds Stop physical proof to the solved artifact's
residual tolerance; strict validation and production candidates are unchanged.
Single six laps pass in253.608948s/penalty0, callbackmax15.511ms/0overruns,
command receipt40Hz/max34.638166ms. However source command timestamps have
11duplicates/backward and two gaps>50ms; odometry/clock delivery pauses225/226ms.
Dev2 rejects at D1decision951/speed1.718736m/s after949selectedStop379.
Both Domains callbackoverruns0/max17.646/21.194ms, commandreceipt40Hz/no gap>50ms;
source timestamps and odometry delivery still have gaps. No coupled finish.
949 captures actual normal365, not379's artifact or951's exact observation.
The terminal snapshot submission suppresses a generic final-authority record,
but the downstream terminal bucket already contains949. Reproduce and repair
that observation gap before guessing current-world Stop authority transitions.
The earlier923comparison has physical Stop witnesses; four explicit candidate
missions show923needs a different accepted candidate from1629, while4264remains
Unknown. No offline formulation was promoted. Runtime is stopped and protected
user artifacts restored. Full acceptance remains open.

Preceding complete-rest baseline ae6862aa and its923/328/925failure:
The complete-rest candidate producer is implemented and locally validated:
25packagebuild,60CTestgroups/2368local records pass. Same-world1629production
join/command passes;4264still rejects. Single-r1finishes6laps252.778778s/penalty0,
moving override0,callbackmax18.373ms/0overruns,command40Hz/receiptmax39.216995ms.
Dev2-r1rejects: D2first visible moving Emergency925at1.823493m/s; no finish.
D1callbackmax31.967ms/3overruns, D2max22.277ms/0. Both command streams40Hz and
no gap>50ms. One causal observation warning per Domain is startup at0m/s
(D1spawned,D2before state observation), not a moving-race failure.
Actual async Stop publication is confirmed near rest (D1source308/decision844,
D2source309/decision827). Moving multi-tick Stop/restart acceptance stays open.
First terminal snapshot is923(fp8706044921014371466) with normal328published
source, not925world and not alternate395. At923alternate395/908joined/selected;
its final command/artifact is not inferred from that log. At925normal328has
cursor2.1s/pose mismatch1.430411m/current control speed1.883513vs expected3.224031,
only publisher-interval continuation and rejected terminal wall proof.
All runtime is stopped. Preserve before/after/rejected outcomes and investigate
source switching/current-state proof plus callback overrun before another run.
See [complete-rest results](../20260909-mpcc-complete-rest-candidates/results.md).

The preceding complete-suffix consumer remains locally validated. On its old
world1629capture, conditional one-solve188tick replay reached rest (170solved
suffix/18existing generated terminal). Preserve this as old offline evidence,
not runtime evidence for the new producer.

A separate Stop pose producer displaced world968's body37.925mm; its stale
projection/bundle coordinate is repaired. The corrected successor correctly
rejects wall. Native wrong-pose150mm and25ms-boundary regressions are preserved.
Final67retained cases/60CTestgroups/2366records and25packagebuild pass.
Single-r1finishes6laps253.323883s/penalty0, moving override0, callbackmax12.918ms
and command receiptmax34.149647ms; source gapmax35ms/no duplicates in this run.
Dev2-r1rejects D1decision4264at8.72m/s: terminal wall reject13 and independent
Stop wall reject, while actual3695is atomically preserved. Retained wall proof
only covers the publisher interval, so it cannot discharge the new terminal
condition. No finished race. Each Domain88reportedwindows,3564/3573cycles,
callbackmax20.292/20.403ms and0overruns. See [complete Stop proof results](../20260909-mpcc-complete-stop-suffix/results.md).

Previous tangent fix8d6ebf8apasses its own single6laps but dev2world968failed.
All164maximum-brake profiles and8rear-QPmethods remain rejected;3of4free-rest
methods are offline positive witnesses. Actual1100/world1629and all older
missing-artifact/solver/clock failures remain separate evidence. Earlier
source/sensor delivery pauses are not erased by the new single clean clocks.

The preceding paired longitudinal observer/clock commit6c4875ed passes one
single six-lap trial253.768982s/penalty0, moving override0, callbackmax16.564ms,
control receiptmax37.079096ms. Its dev2D1world4428terminal wall rejection remains
recorded with missing actual3834artifact and one D2callback overrun28.758ms.
A/B/C/Dcomparison on4428found no accepted bundle. No substitute artifact is
permitted. Single-r1clock-owner rejection, r2Unity startup stall and source/
clock delivery pauses remain in [longitudinal results](../20260909-mpcc-committed-longitudinal/results.md).

Observation slice03e174e5preserves previous actual Stop453/world993. Original
Stop and native current-world rejection are reproduced without re-solving.
Absolute command replacement is rejected across clean single data. The selected
paired observer retains measured-vs-commanded response and passes the old
same-world terminal proof at1.420476m/s. This closes the local producer defect,
not the new integrated failure. Prior same-world A/B/C/Dand fresh Stop arms
remain rejected without physical infeasibility proof.
See [published Stop audit](../20260909-mpcc-published-stop-audit/results.md).
Older dev2 loss 978 and missing Stop 442 remain historical failures, not replaced.
See [side-peer/Stop results](../20260909-mpcc-side-peer-stop/results.md).
The exact same full-body source now admits a native production zero-objective
Stop with unchanged hard constraints, approximately 8.0 ms offline. Old normal A
still fails. New dev2 normal source 381 has an exact rational certificate that its
affine convex branch is infeasible even with current row tolerances; this says
nothing conclusive about physical-scene feasibility. The older normal A's earlier
invalid tangent is repaired with the model adapter's existing declared-box rule.
The raw-primal initial course-window boundary is repaired
with a required separate semantic physical state:25packages/60CTestgroups/
2339records pass. One single trial finishes six laps253.944031s/penalty0,
active moving override0, callbackmax18.913ms/0overruns, receiptmax35.070181ms.
Same-run source/clock publication pauses remain separately documented.
See[initial-state results](../20260909-mpcc-semantic-physical-initial/results.md)
for the earlier zero-objective failure and its baseline evidence.
The first complete-bodydev2run is
rejected with moving Emergency/Recovery and one lap per vehicle. See
[peer results](../20260908-peer-envelope-audit/dev2-results.md). Do not advance
to a submission pass claim or repeat this unchanged trial.

## Completed work

- Astra harness: root/package AGENTS,7Skills,project Astra/high default;
  difficult MPCC audits use an explicit xhigh override. Frontmatter/TOML/links
  and effective configuration checked. No reasoning quality/cost benchmark claimed.
- Semantic target time, Stop feasibility/reference and certified publication,
  stateless successor provenance: local causal repairs and regressions recorded
  in their steering/central experiment registry. Integrated coverage remains open.
- Local AWSIM calibration: UNKNOWN GNSS covariance explicitly configured,
  nominal ego body enclosed by measured footprint, omitted static wall cells
  added and native/editor preservation aligned.25packages build and regression
  evidence retained. None of these alone establishes race acceptance.
- New bounded observation preserves actual published solver source, artifact
  and publication clock before Emergency clears the ledger. No new authority.
  Latest25-package build passes;60CTest groups/2270package-local test records,
  0errors/failures/skips. All7normal intents preserve original solver source.
  Final bounded diagnostic captures actual published source6782at failure7405.
- Coordinate repair now integrates physical Cartesian motion and projects into
  the exact immutable reference frame, retaining the7-state solver order and
  unchanged actuator model. Native invariants24/24pass;25packages build and
  all60CTestgroups pass. Old model artifact identities are rejected. Fixed-control
  source6782error against independent Cartesian ODE is0.231333mmmaximum,
  previously651.9642mm. New solves of6782/7405stillreject wall rows; no
  physical infeasibility claim. Initial6laps252.253662s/penalty0, no active-race
  moving override or Recovery. Callbackmax20.390ms/0overruns, control receipt
  max39.862633ms/0gaps>50ms. Source duplicates coincide with simulator clock
  and sensor publication pause. Later geometry-lineage refusal checks and
  normal/continuation/Stop binding regressions pass; final build25packages,
  all60CTestgroups/2335colconrecords,0errors/failures/skips. The integrated
  campaign with those final checks and peer geometry remains open.

## Current evidence

Corrected-wall trial20260908-wall-map-single-r1:6laps253.198868s/penalty0,
but moving Emergency10699/10700at7.53/7.55m/s. First rejection is delay-prefix
collision at0.105s, before continuation or Stop-terminal proof. The old exact
published10073artifact was not recorded. Fixed-input comparisons do not find
a feasible alternative or prove physical impossibility. Contacted grid cells
contain wall faces at body height, not just overhead geometry.

Observation trial20260908-execution-evidence-single-r1:6laps250.508286s/
penalty0, no active-race moving override, callback20.701ms/0overruns,
control receive max35.215855ms/0gaps>50ms. Failure did not recur, so published
artifact capture was not triggered. The protocol permits at most2additional
same-settings diagnostic trials, solely to obtain the missing failure data.
The earlier rejected result is retained. No pass can replace its explanation.

Observation trial2finishes6laps252.718750s/penalty0 but loses normal authority
at9005/9006at7.65m/s. Here delay prefix and20-stage continuation are clear,
while both terminal Stop references fail. Original source is absent because
the source producer only retained ShiftOut/Pass inputs. A native probe confirms
5of7normal intents missing. The observation producer now retains all7; its
new build/tests and final planned diagnostic now pass observation acceptance.
This does not fix the earlier delay-prefix or terminal failure by itself.

Observation trial3finishes6laps259.300171s/penalty0, but Emergency7405/7406
at7.53m/s. Source6782and actual artifact/publication clock are saved; first
rejection is delay-prefix collision at0.120s, before continuation or terminal
proof. Callback21.207ms/0overruns;receive max35.177231ms/0gaps>50ms.
The three-trial observation campaign is closed, with all outcomes retained.

The actual native transition fails physical coordinate invariance: stationary
body, curvature0.2/m,lag-0.3m,virtualspeed4m/s moves39.9899mm in0.1s.
14of24native analytical cases fail. With captured6782controls, native world
position differs from independent Cartesian motion by47.0783mm at45.123ms.
Same source grid/native sweep: native path clears hard reserve; Cartesian
path rejects at the first sampled4.512msboundary while body margin stays
clear. Correct arc-length equations alone still leave16.0719mm at45.123ms,
because stored curvature and piecewise reference-frame derivatives disagree.
Complete actual-frame equations agree with Cartesian integration offline.
This was a proved model/certificate defect; the current local repair is above.
It is not yet a complete attribution of the live delay-prefix failure. Full-horizon rollout
is conditional: Emergency supersedes actual publication aroundcursor0.05s.

## Order to finish

1. Complete current-world full-rest candidate generation and async promotion
   using the sealed witnesses and new world4264, with all-peer physical proofs.
   Observation bundle and longitudinal observer/clock slices are validated. Coordinate-consistent integration, semantic initial state,
   whole-body peer enclosure and Cartesian peer-plane repairs are implemented
   and committed. Do not repeat their completed native/build work without a
   changed input or newly discovered concern. Preserve the unresolved Stop
   successor course-window boundary and the unmeasured longitudinal phase.
2. For any new physical-versus-solve disagreement, seal the executed artifact,
   current world, serialized inputs and independent observations, then perform
   the bounded architecture comparison before another patch in that family.
   Do not replace wall/peer evidence with margin, delay or solver tuning.
3. Fresh fixed campaign: single six laps, Follow, both Pass/Return directions,
   certified multi-tick Stop/restart and Recovery/Rejoin. Then async identity,
   dev3/dev4 and gate1/2/3. Seal all inputs/binaries and all repeated results;
   check final authority, wall/peer proof, callback/publish timing and every vehicle.
4. Submission: create tar.gz, bake eval image from that artifact, run make eval;
   verify top-level archive/interface/schema and same-run outputs. Update
   registry and P0-P4 tasklist with actual evidence before declaring completion.

Detailed current work: ../20260908-mpcc-delay-prefix-audit/ and
../20260909-mpcc-coordinate-consistency/ and ../20260908-peer-envelope-audit/.
Full acceptance remains P2partial/P3/P4open.
