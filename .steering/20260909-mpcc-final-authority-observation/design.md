# Final authority boundary observation

Continue the authorised autonomous MPCC work after local commit `e5b8c455`.
Production authority remains unchanged during this observation slice. No
simulator/build currently running. Rollback is e5b8c455; preserve the previous
405 artifacts / 75 snapshots / 163 experiments and all protected user files.

## Earliest demonstrated observation defect

Run `20260909-stop-provenance-dev2-r1`, D1 first terminal world **949** records
normal **365**. The final moving Emergency is **951**, after Stop **379** was
selected at 949. Its exact artifact/current physical observations are missing.
The same gap occurred at old D2 923 / normal328 / selected Stop395 / Emergency925.

`rate_resolved_normal_production_control` treats
`terminal_snapshot_submitted == true` as evidence that this decision is already
recorded and skips `record_rate_resolved_normal_authority_failure_snapshot`.
But submission only enqueues a `LatestOnlyWorker` job. Later, the recorder's
first-failure bucket deduplicates 951 against 949. The generic final failure is
therefore suppressed before publication clears the live ledger. The existing
source-contract test even asserts this obsolete suppression condition.

A second independent ownership risk is that LatestOnlyWorker deliberately
replaces pending work: receding planning wants the newest job; first-failure
evidence wants the earliest observation of each bounded category. Do not change
the solver worker's semantics. Establish a deterministic blocked-worker native
probe of the recorder's current scheduling, plus actual recorder bucket calls,
before choosing the smallest isolated observation owner.

## Intended bounded repair, subject to the native evidence

A prior terminal diagnostic must not suppress the later final authority-loss
category. Freeze terminal diagnostics at their existing boundary, then always
inspect final loss after admission; a decision may have both distinct events.
Preserve the earliest event in each fixed
intent/homotopy/boundary bucket before asynchronous I/O. Use a bounded first-event
queue or equivalent isolated observer; no receding solver policy/timeout changes.
Preserve current terminal viability diagnostics and immutable source lineage.

The final record also needs exact revalidation inputs that cannot be recovered
from a bag: measured speed, current-time physical steering, committed/control
steering age, control-origin speed/response, actual course association/path
length and the inspected Stop plan with its execution clock. Keep the latest
published artifact distinct from a rejected/inspected candidate. Extend the
internal diagnostic schema compatibly; no changed evaluation JSON/ROS contract.
Validate optional evidence and write explicit missing/invalid status; never
invent an artifact or let bad supplemental evidence discard a valid world.

Native fixtures must cover distinct terminal/final boundaries, duplicate final
events preserving the first artifact, a busy writer, no I/O on the callback,
invalid supplemental evidence, and exact serialization roundtrip. Retire the
old suppression and latest-only evidence path in the same slice; keep every
existing control, constraint, proof and publisher unchanged. Build/package,
recorded replay, fixed runtime, seal/register/spec/status/local commit follow.

## Remaining causal work

949 has terminal peer clearance -0.00122436 m against d2. At951 the retained
Stop379 reports PublishedPlan cursor0.600000 / firstpublish10.360000 /
firstcursor0.550000; predecessor steering-0.128003 vs expected-0.010359,
reachability bounds[-0.155977,-0.100028] over0.015s. It also fails terminal peer
proof; independent Stop from365 is wall-clear but peer-blocked. Do not conclude
the switching cause before capturing exact source/artifact/current request.

The new run has no callback overruns; old D1's21.954ms Recovery safety maximum
was only a window contributor, not a proved root cause. Single and coupled
source/observation delivery pauses remain separate unresolved evidence. All
remaining moving Stop/restart/intents/dev3/dev4/gates/submission acceptance stays
open. This recording repair cannot by itself close race acceptance.

## Baseline native observation

`output/20260909-final-authority-scheduling` uses the existing native snapshot
fixture plus actual recorder/LatestOnlyWorker implementations, not the controller.
Synthetic terminal event1001is written; event1002in the same bucket is Duplicate.
An explicitly invoked generic final event1002is Written. A deterministic blocked
worker accepts the first pending final job, but a second pending final job
replaces it; first_final_executed=false/later_final_executed=true. This confirms
both scheduling primitives behind the live missing record; it does not substitute
synthetic fixtures for949/951or assert a control root cause. The baseline probe
has no production edits or publication authority. All build/native logs retained.

Actual365was extracted without solving to
`output/20260909-final-authority-365-original/snapshot.yaml`, including its own
published-wall-grid.bin. The source fingerprint is6085679935950841233.

Implementation direction: an isolated first-event recorder should reserve a
fixed intent/homotopy/boundary bucket at enqueue time and drain distinct buckets
in FIFO order, leaving LatestOnlyWorker unchanged for receding planners. Terminal
diagnostics and final authority loss are different categories; preserving both
in the isolated queue removes the old reason to suppress a second submission.
For exact revalidation evidence,
capture the Request that was actually passed to retained::evaluate on rejection,
rather than rebuilding it later after another dynamic observation or intent join.
Keep inspected source/artifact/clock separate from the latest published source.

The initial isolated recorder now passes all15native snapshot tests, including a
blocked callback/queued final event, duplicate suppression before replacement,
invalid world not occupying a bucket, exact first artifact and drained shutdown.
It is wired into the controller; the obsolete terminal-submission guard is
removed and all104source-contract tests pass. Full snapshot hashing remains on
the recorder thread. Exact request serialization and build/runtime verification
are still pending; the15native result predates the final hashing ownership move.
