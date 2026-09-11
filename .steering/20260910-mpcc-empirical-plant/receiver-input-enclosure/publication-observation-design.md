# Final publication failure observation

2026-09-11 JST. Baseline/rollback7c283cc7. M1–M6 remains open.

Dev2-r30 rejects before race readiness. D1 first post-publication failure645:
nominal2.529999943, before2.554999942, after2.559999942, deadline2.554999943.
Its callback is27.305793ms, primary proof21.471ms and Recovery region3.669ms.
D1failure680 is22.163763ms in steady time but about30ms in ROS publication time.
These clocks are distinct; neither may be substituted for the other. R197's
numerical improvement is insufficient for integrated deadline acceptance.

The detection gap repeats r29: an accepted physical proof can later fail at the
publisher, outside the existing normal-authority-failure capture. Its immutable
Certificate already owns the exact Request. Expose that pointer for observation
and queue it with the certificate, nominal/decision/before/after raw clocks and
actual float packet when the existing final guard rejects. Never reconstruct
the input from a later callback, overwrite the nominal source, or backdate history.

A separate bounded recorder preserves the first pre/post-publication failure
in each stationary/moving bucket (four total). It drains on shutdown, and all
serialization, grid writing and callbacks run off the control callback. The
new internal observation schema labels the inspected source and exact current
request separately, and records mismatched associations explicitly. It adds no
control authority, retry, hold, clock tolerance, output suppression or solver.
The existing failsafe executes before enqueueing this diagnostic observation.

Validation: exact immutable request/grid/packet/clock round trip, first-event
preservation and shutdown drain, missing/invalid association reporting, package
build/tests. Then committed dev2-r31 collects the first failing final boundary
for replay and timing attribution; it is not a race acceptance substitute.

- [x] Preserve dev2-r30 failure, callback clocks and protected artifact restoration.
- [x] Add bounded asynchronous final-boundary observation and regression tests.
- [x] Buildr63/testsr52:26packages,2464records/63groups passed; evidence/specification prepared for local commit.
- [ ] Fresh committed run and exact failing-input replay, then causal repair/M4–M6.


[Sealed evidence](publication-observation-evidence.json) preserves the failed
r30run and the observation-only implementation. The permanent regression checks
all four first-event buckets, worker-thread callbacks, shutdown drain, original
request clocks, exact float programme and byte-identical current wall grid.
Missing certificates and mismatched request decisions remain explicit evidence,
not falsely relabelled sources. Next: fixed committed dev2-r31.

Dev2-r31on3387e684captured six exact final-boundary inputs. R207replays all six
Accepted, including full-rest programme/materialization/join, in9–13ms. Live
D1moving863uses19.056msprimary and5.502msRecovery region (25.278mscallback),
with35msROSclock advance. D2moving883uses22.772msprimary and4.812msRecovery.
Single evaluation per cycle in nearby retained telemetry rules out assuming a
duplicate candidate evaluation as the missing cost. No timing fix is claimed.

D1stationary602also demonstrates a separate endpoint representation issue:
nominal1.619999963 plus.025 rounds to1.6449999629999998, while the received
nanosecond clock converts to1.644999963. Existing exact-double guard rejects it.
Do not silently add tolerance: time representation and the physical enclosure's
outward publication population must be reconciled explicitly. Moving30–35ms
violations remain real failures regardless of that endpoint issue.

Next bounded diagnostic dev2-r32keeps3387e684control code/config and observes
the two SingleThreadedExecutor main threads through read-only/proc schedstat.
Capture CPU-runtime/runqueue counters at1ms and bound deltas over each logged
callback's steady duration and wall completion. This is diagnostic load, not
race acceptance. No affinity, scheduling policy, kernel setting or controller
parameter changes. Kernel perf_event_paranoid=4precludes ordinary perf events;
no system setting is changed. Use the measured CPU/wait split to choose the next
repair instead of treating all live/offline time differences as arithmetic cost.


R31succeeds at observation capture, not deadline acceptance. R207/r208 preserve
all six original physical positives with no authority. R32is inconclusive due
to startup sensor staleness. R33fails beforeReady and measures startup control
CPU/runqueue work; itsReady event arrives after shutdown initiation.
[Exact-runtime comparisons and next production slice](exact-runtime-design.md)
record the falsified request/adapter-cost hypothesis and bit-identical candidates.
