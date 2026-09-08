# Local verification passed; three-repeat dynamic acceptance failed

New regression fails before selection repair on both the full Stop lateral
trajectory and final steering (-0.071895 vs+0.074767rad). All64 retained tests
pass after repair, including unchanged feedback/publisher/state/world/peer
checks. Source ownership contracts104 pass. Full make autoware-build finishes
25 packages in4min30s; full package2318 tests,0 errors/failures/skips.

Controller SHA-256:
5b65133ad69c424ff3c1954cc73b778b9d9aef283aff0d05586ac90f6ef0ae96
Comparison SHA-256:
9bcb744f18e40d810373e02fb3fff7c91e0720d4c1b3ef4147d8eeab5b684826

The temporary live counterfactual call is removed. The diagnostics-only API
remains for bounded-profile/world-identity regression and returns no Proof or
command. Selected Stop reference and attempt count are in existing telemetry.
No profile extrapolation, new timer, new command store, or relaxed physical
proof is introduced. Failed reference computations are included in runtime
breakdown totals.

Single acceptance uses protocol.json: three identical trials at most, stop
campaign on first failed completed trial. Configuration and binary hashes are
checked before launch. Each trial has its own manifest, logs, bags, result JSON
and preserved binaries. Overall integrated acceptance is not complete.

## Fixed-binary single trial 1

`20260908-mpcc-stop-reference-single-r1`: six laps249.293030s, penalty0.
Laps44.295258,41.449650,40.809513,40.944542,41.454651,40.339413s.
No active moving Emergency or Recovery. The two moving Emergency logs are
decisions11190/11191 after AWSIM finish at1788797997.965308234. Startup's
provisional start/ready reset remains visible and separate from the real race.
Controller's own lap-complete logs stop at5 because AWSIM finish precedes its
last seam detection; formal v3 vehicle results confirm6.

Callback max20.905ms,0overruns across13092 reported cycles. Same-run MCAP
10466 control messages/261.582180s receive time: mean24.995908,p9526.324940,
p9927.160835,max33.506155ms,0nonpositive/0gaps>50ms. Source stamps have
21nonpositive gaps/max99.999998ms; distinguish them from receive timing.
Evidence is sealed in single-r1-evidence.json.

## Fixed-binary single trial 2

`20260908-mpcc-stop-reference-single-r2`: six laps250.933380s, penalty0.
Laps44.605324,41.509663,41.079571,41.309620,41.179592,41.249607s.
No active moving Emergency/Recovery. Finish observed1788798357.134115088;
only subsequent moving Emergency decisions11249/11250 appear.
Callback max18.910ms,0overruns across13060 reported cycles. MCAP10527
commands/263.099625s receive time: mean24.995214,p9526.225030,p9927.105033,
max32.498837ms,0nonpositive/0gaps>50ms. Source stamps have12nonpositive
gaps/max99.999998ms. Evidence is sealed in single-r2-evidence.json.
Trial3 used the same verified binary/config/map and failed below.

## Fixed-binary single trial 3 — rejected

`20260908-mpcc-stop-reference-single-r3`:600s timeout, finished=false,
5completed laps207.678345s. Wallpenalty1 onlap6/race242.938812s;
aggregate8.110138s, event8.115005s. Completed laps44.360271,40.794510,
40.494446,40.979549,41.049564s. Callbackmax18.061ms/0overruns across
29898 reported cycles.24190 MCAP commands/604.674902s receive time:
mean24.997929,p9525.879383,p9926.637135,max33.190966ms;0gaps>50ms
and0nonpositive. Source stamps32nonpositive/max99.999998ms.

Vehicle-status speed drops7.796716→0.852230m/s over35ms at source250.6s
while command speed stays7.80m/s with positive acceleration. Normal proof
remains accepted through the low-speed state. Recovery10713 precedes the
subsequent authority loss10714; the latter is not the first cause. GNSS vs
EKF position and actual simulator/map correspondence remain to be resolved.
The initial event and the later wall penalty are not yet equated.

The campaign stops at its planned3trials with2passes/1failure. Prepared dev2
has not run. The dedicated follow-up is
`../20260908-mpcc-single-physical-stop-audit/`; all hashes and results are
preserved. Runtime partial-bag parsing began after the failure, so full-run
timing after that point is not an isolated CPU benchmark. No concurrent build
or solver replay ran during any of the three single trials.
