# Six laps finished; safety acceptance failed

Same-run result: six laps,266.276641846s total. Laps44.325264,41.509663,
41.289616,41.314621,40.694489,57.143002s. One wall penalty on lap6,
race_time254.377243s. Aggregate penalty8.210140s and event duration8.215042s
are recorded separately as emitted by the evaluator; no schema is changed.

At decision11046, source observation260.744994s/control260.874994s,
retained sequence10415 loses terminal viability. Delay path and publisher
interval remain clear; the longer continuation and track-reference Stop both
hit the wall. Position/yaw join errors are0.021878m/0.002012rad. Generic
Emergency is published. Stuck recovery arbitration appears at11185; certified
Cruise resumes at11464 with sequence10561. The final lap then completes.

Receive-clock control intervals:11134 messages over278.303369s,
mean24.998057ms,p95 26.312876ms,p99 27.147751ms,max33.913851ms;
zero gaps over50ms and zero nonpositive gaps. Source stamps have11 repeats
and max79.999998ms, which are not publish timing failures. Reported callback
maximum17.439ms, zero25ms overruns over13943 reported cycles.

Artifacts: `output/20260908-mpcc-single-six-lap/{bag-analysis,log-summary}.json`,
`d1/{result-summary,d1-result-details}.json`, log/MCAP and failure snapshot.
The failure is sealed in `../20260908-mpcc-single-stop-horizon/manifest.json`.
The short Stop replay briefly overlapped the late portion of this run, so no
isolated machine-load benchmark is claimed. This is the mounted dev workspace
using evaluation.launch.xml, not a baked submission-image acceptance run.

The monitor stopped only mpcc-single6-20260908 after recording finish/results.
Parent P3 single-vehicle safety acceptance remains unchecked; continuing into
dev3/dev4 or packaging now would not repair this established failure.
