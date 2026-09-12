# Early and complete starting populations

2026-09-12. Baseline / rollback 660227c9. Preserve the sole C5 normal dispatcher,
original source identity, actual prior/suffix history, generations, current-world
proof, receiver250ms, publication25ms, control origin130ms and steering100ms.
No tuning of physical bounds, solver budgets, timing rates or grace is proposed.

Standard r64 fails before publication at D1 dispatch957 and later after sending
at D2 dispatch1065. R396 exactly reproduces both original source/current proofs
and before/after rejection. D1's complete relative body/time population is inside
its membership domain but intersects the current wall (DomainUse7); full current
point proof takes29.03ms wall /25.33ms CPU live. The body population includes
future speeds through rest as possible early starting speeds. D2 instead passes
its domain; most of its9.405ms callback is off CPU. Scheduling is a separate
remaining cause. Both cars are almost stationary and no laps complete.

R397 compares four independent representations on five frozen worlds:
- Full relative body/time population: current production.
- Early original time interval, keeping the full body population.
- Early interval and source body hull through that interval, including raw state.
- One original time bin and its source body hull, including raw state.

Bin width is the reserved prior packet count plus the current packet, expressed
using original publication epochs. No old future point is translated/reused:
every alternative is freshly propagated with the same complete original input
memory and exact physical/current-world checks. D1 r64 still rejects with early
time alone; early body/time passes (316samples,16.5ms worker /1.94ms world offline).
Time bins also preserve the late r64D2/r60D2 cases; early-only cannot cover their
times. All bins precomputed would add unmeasured worker cost and are not promoted.
R63D2 remains outside the lateral-velocity population; early variants also miss
yaw rate. This proposal does not solve that separate body-coverage failure.

Retain the complete proof and additionally build an early body/time proof for
source-horizon programmes. Try the early proof before the complete one, then
retain the existing mandatory complete current point proof if neither applies.
The early end is the original window at index first_suffix_index, corresponding
to the reserved prior count plus one. Its body hull uses original source samples
only through that end, with the raw source body state included. Every member and
time is independently proved through rest before the evidence becomes available.

Represent the early evidence as an immutable private child with the same exact
certificate. Return the evidence actually checked, not its parent, to
CurrentDomainProof. Existing physical fingerprint, deadline/rest checks and
snapshot encoding then use that selected evidence's own numerical proof. Never
hash or check the parent's rest horizon when the child was selected. No new
caller-provided token or normal authority is introduced. Follow and short
programmes keep the existing path. Full late coverage is retained.

Validation required: complete original-source replay and early/current-prefix
parity, early-to-complete transition with exact actual history, foreign/history/
context/world/clock negatives, full package/build and standard committed run.
Additional worker cost must be measured; offline proof cost is not a runtime
bound. Current body coverage and scheduling, all M4-M6 and final submission/eval
remain open. R64 source/record clock windows are separate-subscriber observations;
recorder receipt must not replace the controller's after-publication guard.

Implementation retains full relative evidence and adds one private early child
for selected source-horizon programmes. The actual checked leaf owns current
proof identity and rest checks; existing snapshot encoding follows it. The final
package suite passes all160 C5 tests,2,584 records/66 groups (source105), and
build107 passes all26 packages. Native398/399 preserve two fixture-assumption
failures corrected before final validation. R400 replays12 historical scenes:
r64D1 selects early evidence and passes the current wall; r64D2/r60D2 keep the
complete late proof; r63D2 remains outside the body population. Original source,
actual prefix/footprint, selected-leaf ID and all late guards remain exact.
[Evidence](starting-domain-window-evidence.json). Standard committed r65 and all
remaining runtime and M4-M6 acceptance are next.

R400 r64D1 offline selected-current median is2.062468ms across50 repeats;
worker full+early construction is47.564606ms. R64D2 keeps the full leaf with
0.4802855ms current median. These separate-process measurements are not a live
bound. The recorded D1 before-clock remains rejected, as its real deadline had
already passed; no captured timestamp is moved to claim a successful send.
