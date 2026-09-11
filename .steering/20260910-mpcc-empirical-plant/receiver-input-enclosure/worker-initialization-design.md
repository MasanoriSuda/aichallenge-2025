# Worker initialization order

2026-09-12. Full package test90 stops in the existing
ReportsJobFailureAndInvalidTicket test. BoundedSingleJobExecutor declares its
thread before ticket, stop and statistics fields, then starts it in the member
initializer. The child can read stop_requested_ before its initialization. With
reused stopped object storage, that read can observe the old true value and exit;
a later accepted submission then has no worker to complete it. This is also an
unsynchronized read during initialization.

R383 widens that permitted interleaving only in a diagnostic header copy with a
5ms initializer pause after thread startup. Original construction loses12/12
jobs; starting after all members completes12/12. The unmodified test90 timeout
is preserved separately. The production fix starts std::thread in the constructor
body, after every member is initialized. No delay, timeout, queue or retry is added.
LatestOnlyWorker was inspected: its thread is already last, so no change is needed.

A native regression reconstructs stopped storage500 times and checks normal and
exception completion, fresh ticket1 and reset statistics. R384all5 tests pass.
Build104all26 and full MPCC test91all2582/66 pass, including the pending independent
relative-domain change. Those full logs describe the combined working tree; r384
is the independent executor validation. No claim is made that this race caused
r62's particular scheduling or vehicle behavior. Runtime acceptance remains open.
[Evidence](worker-initialization-evidence.json). Baseline/rollbackbd9515ec.
