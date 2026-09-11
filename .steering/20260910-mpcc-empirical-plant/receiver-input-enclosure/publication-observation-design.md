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
