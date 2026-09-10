# Attribute the remaining callback deadline overruns

Baseline `f1f213fc`, frozen run `20260910-nine-state-dev2-r5`. No moving external
Emergency/Recovery occurs before teardown. The monitor instead terminates on
the tactical Pass → Recovery label at D2decision1852. The actual publisher
keeps certified Pass until fresh certified Rejoin1859, then phaseIdle at118.

The progress watchdog measures gain on the locked target, not ego standstill.
D1 configured40km/h, D2normal15/start20km/h. Public same-time positions show
D1travel84.979m/D2travel46.198m during Pass; diagnostic course gap35.981→75.435m.
The public planning path includes a33.042m closing segment from its garage
prefix, so these host course projections are diagnostic, not live projector
replay. Positions/velocities and live locked_s independently confirm retreating
relative progress. This is a legitimate tactical abort to canonical Rejoin;
the watchdog, speed caps and Recovery supervisor must not be disabled/tuned.

Campaign classification now records this exact watchdog transition separately.
Every moving external Emergency/Recovery, every other unexpected Recovery
entry, launch failure and completion/penalty criterion remain. The incomplete
r5run remains incomplete; it cannot become a six-lap pass by reclassification.
Intentional physical Recovery tests remain separate from this normal Rejoin.

Actual unresolved timing: D2raw2185callbacks, p95 21.395014ms,
p99 29.77800624ms, max46.989231ms,7adjacent-overrun pairs during ShiftOut
at1350–1373. MPC costs14.466–28.469ms, Recovery safety2.333–8.793ms,
publication1.35–18.122ms in those pairs. Marker cadence matches some publish
spikes but causation is not yet measured. Later Pass1771andRejoin1895 have
one retained evaluation each, with exact wall proof accounting for much of
23.239/30.569ms join time. No certificate/solver/rate/timeout changes are justified.

Add observation-only clocks around problem initialization, publication-successor
creation and prediction markers, retaining point counts in the existing overrun
detail. Total callback remains measured by its original clock; nested costs
are not double-counted. No telemetry is removed or throttled further.
Run a bounded timing diagnostic until the first adjacent overruns (or the
existing earlier failure). Preserve the same control settings and all evidence;
do not count the observation run as race acceptance. Next fix must address the
measured producer while preserving its outputs and hard proofs.

Validation: buildr17passes26packages in4min10s; testsr13passes2394records,
0errors/failures/skips. Exact monitor-block replay against preserved pre-teardown
r3/r4/r5logs retains both moving Emergency rejections and records the one r5
tactical Rejoin without mistaking it for an external override. The initial
classification probe included post-teardown missing-input Emergency2025; the
corrected probe uses each run's recorded shutdown boundary. No run data is
edited or discarded. R5 remains incomplete and timing-rejected.

The timing diagnostic waits for adjacent overruns after an observed ShiftOut
entry, avoiding unrelated startup samples. All existing earlier real failure
conditions remain active. Command source duplicate intervals in r5D1/D2are0/1,
backwards0, receipt max36.355/52.506ms; no invalid command or serialization join
rejection. The relationship between timing outliers and stale execution is not
yet established. Marker/proof costs still require the new observed run.
