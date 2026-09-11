# Scheduled node integration (working, baseline a8c70824)

Earliest established invariant failure: r44 D1decision892 actual publication
before/after exceeds the original25ms window while synchronous proof consumes
that window. C3/C4/C5boundary native evidence does not establish runtime acceptance.

Replace synchronous normal, Stop-lattice, Stop-successor, Gate-A and previous-
intent authority joins with one scheduled programme dispatcher. Solver/tactical
candidates remain immutable inputs; no diagnostic proof is executable. A worker
captures the actual post-send observation/history, its unchanged next integer
appointment, current world and real source generation. Main-thread adoption
checks original measurement population, every actual prefix send, current semantic
context and all remaining original world ranges. Final publisher checks actual
raw clocks and exact payload/source/index before and after every send.

Post-send prediction must rebuild pose/progress/speed/tire/body/desired steering,
predecessor age and prospective prefix. No proposed command enters actual history.
Follow must bind its actual source epoch and course branch when querying later;
changing an observed timestamp alone is invalid. Published programme context is
committed by a real send, separate from the newly proposed tactical intent.

Keep model, bounds, receiver250ms profile,25mswindow,130ms nominal origin,100ms
nominal steering delay, iteration/budget/timeouts unchanged. A failed boundary
remains a rejection and any already sent packet remains in the ledger. Recovery,
reset and policy changes revoke old work. No new positive hold or grace.

Acceptance: focused original-input regression, package test/build, first-failure
runtime capture, actual Stop/rest/restart and all intents, then same-finalHEAD
M4–M6/submission acceptance. Rollback base: a8c70824. No node authority promotion
has happened until the worker and sole publisher are connected in this slice.

Implementation checkpoint: node now selects only scheduled dispatch candidates;
old synchronous authority joins have no callers. Their unused live Stop-lattice
worker enqueue is removed. Post-send capture uses the actual observation/history,
and later Follow queries project the fresh Cartesian target under its original
course-branch association. Source job and current dispatch IDs remain distinct;
final identity accepts the pair only with opaque actual-publication evidence.

Buildr82 failed because the new Follow projection lacked a transitive V2X link;
that dependency is now explicit. Buildr83 passed26packages/5min11s; testsr71
passed2521records/64groups,0errors/failures/skips. The104structural source tests
now check scheduled production edges; retired joins cannot be restored silently.
Native r277 passed129tests before the Follow/receipt additions. R278same-file
copy and r279missing fixture include are diagnostic setup failures, not control
failures. R280 reproduces a clock-owner error: proof creation1.05, appointment1.1,
current callback1.11, actual send1.115within deadline1.125. Supplying the callback
as proof-creation clock rejects; using the immutable original creation time
accepts. Dispatch now reads creation time directly from its certificate, so a
caller cannot substitute it. Actual before/after windows are unchanged, with
new1ns-outside regressions. R280pre-change sources are preserved under
output/20260911-nine-state-scheduled-node-integration/r280-before-clock-binding.

Buildr84 passes26packages/5min6s; testsr72 passes2522records/64groups,
0errors/failures/skips in32.75s. Native r281 passes132tests/839ms.

## Dev2-r45: failed integrated acceptance

The frozen a8c70824working tree/buildr84 produced20D1 and25D2 authenticated
scheduled sends, all before Ready. First D1dispatch563/job562/source13 at
nominal0.609999986/rawbefore=after0.619999986/deadline0.634999986 has a matching
final receipt. No suffix index1 send, no Ready/Start normal authority, no laps,
and no live Follow coverage. The stopped run is explicitly failed; it is not a
successful timing or race campaign. The runner was interrupted only after four
exact original/current input snapshots (38files) were saved. Containers stopped;
both user JSONs and original DLL hashes restored.

Recorded callback maxima are21.443439/24.270438ms, overruns0/0, with4506/4508
emitted samples including startup/shutdown. Command bag receipt averages
39.99649/40.02484Hz; maximum receipt gaps41.67914/43.82539ms. These stopped-run
metrics do not establish moving acceptance or six-lap timing. Invalid commands0.

Three independent rejection boundaries are reproduced:

1. D1dispatch552/job551: producer computes(progress+dx)+dy while exact frame
   check computes progress+(dx+dy), differing by3.552713678800501e-15m. R283
   reproduces CourseFrameUnavailable. R284rebuilds the producer with one shared
   projection function, keeping the exact control pose; it reaches the next
   prefix gate. No comparison tolerance is added. R285passes133native tests.
2. D1job551 andD2job532: node converts appointed integer ns plus130ms using
   multiplication by1e-9. For474999992/489999992ns this differs by one ULP from
   canonical division by1e9 and fails the exact integer-time API. R286uses the
   same intended ns with canonical conversion; both reach the applied proof
   boundary, where the original startup physical input remains rejected. The
   live producer now uses a shared tested conversion. No appointment, clock
   window, delay, physical input or safety bound is changed.
3. D1dispatch556/job555/source5 andD2dispatch536/job535: old full certificates
   reproduce Accepted in10.2/13.4ms with exactly matching input fingerprints.
   Fresh pose source0.424999990 follows original0.384999991. D1XY changes
   (+6.605318e-6,-2.018314e-5)m, D2(-2.822053e-5,+3.990514e-6)m. The old nominal
   rest tube is a singleton up to roundoff. Fresh measurements are outside it;
   PoseMismatch is therefore a correct rejection under the current contract.
   Velocity/yaw-rate/tire mismatches also occur later. This is separate from
   the two representation bugs, and remains unresolved.

R278same-file copy/r279missing fixture include/r282wrong static versus shared
V2X library are diagnostic setup failures; none is a control-model counterexample.
R280andR283–286 are diagnostic reconstructions, never executable authority.
The diagnostic generation only gives reloaded solver data a local lifetime;
it does not reconstruct or prove live revocation/adoption permission.

## Remaining producer and integration obligations

Establish model/observation error independently from packet-receipt uncertainty.
An arbitrary epsilon, accepting a failed membership check, suppressing newer
sensors or reusing a reanchored population as the old proof is not a repair.
Compare a current independent remaining-program proof and an explicitly modeled
observation/process uncertainty population on sealed input; any new population
must itself meet the unchanged full-rest wall/peer/vehicle constraints. Existing
r193/r194cache rejection evidence remains in force. A covariance is statistical
information, not a deterministic physical bound by itself.

Exact source/current geometry, mission transitions, sibling tactical commit,
Store publication bookkeeping, current-observation-unavailable capture, actual
Stop/rest/restart and all M4–M6/submission acceptance remain open. Old unreachable
sync helper definitions remain for offline diagnostics; their production call
sites are removed in this slice. Do not claim completeness from detached tests.

Final buildr85 passes26packages; testsr73 passes2524records/64groups with
0errors/failures/skips. Native r287 passes134tests/848ms. Source contracts104pass.
Rollback base remains a8c70824. Evidence is in
[scheduled-node-integration-evidence.json](scheduled-node-integration-evidence.json).
