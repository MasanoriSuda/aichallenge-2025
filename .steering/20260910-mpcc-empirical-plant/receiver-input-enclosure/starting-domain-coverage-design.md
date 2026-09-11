# Starting-domain coverage comparison after r55

2026-09-12 JST, baseline4a73f87f. Diagnostic only; production and parameters fixed.
Standard r55 fails D1post974 before teardown: current prefix outside domain,
complete point proof12.980541ms wall/10.509623CPU; entry12.409999722,
nominal12.399999723, before12.424999722, after12.429999722, deadline12.424999723.
Worker26.527ms including12.790075ms domain. Recovery0.050ms. No laps.

Completed whole-run counts (include teardown, not moving acceptance): domain
accepted17/11 and first-packet outside66/41 forD1/D2. Unsupported retained suffix
349/265. New domain accepted current CPU median0.994/0.909ms; worker added time
and large unsupported population mean the live integration does not pass.

Compare same frozen R51/R53/R54/R55 snapshots:
1. Current causal source-observation-through-first-deadline hull.
2. Include original raw source body in that hull (sensor component origins may
   precede source.now). Independently propagate every state/time again.
3. Symmetrize signed components around original raw source body using only the
   original hull's maximum signed excursion, with outward arithmetic. Keep
   forward speed in original nonnegative hull. This is a proposal for a larger
   candidate set, not a sensor tolerance or guaranteed disturbance envelope.
   Independently reprove whole set/time/body/corners/rest/current world. Do not
   clip failed constraints or reuse old future evidence.
4. Oracle current prefix in original frame as a minimal feasibility control;
   explicitly noncausal, never production evidence.

Record exact out-of-domain components and raw/current epochs, original programme
and domain fingerprints, full numerical and current-world outcomes, original
pre/post classifications and no-authority status. No current failure can be
made acceptable merely by enlarging membership; the entire enlarged candidate
must independently satisfy the same full physical constraints. Pending-prior
input memory and suffix>0 remain separate unsupported obligations. Extending
range coverage alone cannot claim full live acceptance.

R328 exact R55 failure: bodyX[-3.9730761000266615e-5,-1.6044569347066515e-6]
is below originalD.Xlower0; tire[.29477926938915106,.2947792693891517] below
originalD.tirelower.29477927505321383. Raw-origin inclusion repairs tire but
leaves Xoutside. All-component symmetry repairs membership but R53full-world
rejects StateBoundRejected, so that arm is rejected. R329symmetrizes only pose
coordinates0..2 and leaves body/actuator coordinates in the raw-inclusive hull.
All4frozen cases then pass whole-prefix membership and independently recomputed
full state/corners/rest/current-world proof. No tolerance or physical bound changes.

Implementation slice, rollback4a73f87f: change only worker candidate D producer,
include all raw source components and symmetric pose excursions using outward
arithmetic, then compute a new full-domain theorem. Current checks, input memory,
original windows, body/actuator limits and complete point path remain unchanged.
Test a fresh pose behind raw source plus all existing negative/Follow/actual-send
checks; exact final replay of all4cases; build/package/source tests; committed
standard dev2-r56. Later suffix coverage and worker availability remain unknown.
Theoretical all-programme adoption-time domain and removing duplicate worker
future propagation are alternatives to compare if measured scheduling still fails;
no such architecture is implemented in this bounded producer slice.

Final native r330all147tests and actual-library r331all4replays pass. Exact
short/full prefixes and original pre/post violation classifications retained.
Buildr95all26packages; testsr83all2561records/66groups, including source105.
[Sealed evidence](starting-domain-coverage-evidence.json). Next committed standard
dev2-r56 tests actual coverage, worker/current timing and deadlines; no race claim.
