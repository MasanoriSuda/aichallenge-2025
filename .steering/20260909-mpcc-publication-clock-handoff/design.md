# Publication clock handoff

Continue autonomous MPCC completion after 17bcf24b. Rollback is that commit.
Use the sealed EKF-repaired dev2 D1 world 937 / artifact 374 and real Accepted
936 request. The 936 request uses the most recent published Bundle clock
(10.009999779000001, 0.3594038730139186); after exact publication 936 the Store
reports the old executed clock (9.779999784000001, 0.12499999699999975).

Earliest suspected invariant: publishing an exact continuation of the latest
Bundle source must preserve the source cursor/time association used by its
proof and command. Identity alone does not restore an earlier execution clock.
Store::mark_executed currently clears the latest Bundle and preserves the old
clock whenever artifact identity equals the old executed plan. Reproduce this
with the real Store before changing production. Replay the same 937 input with
the 936 causal anchor separately; no new solve or offline authority promotion.

Inspect exact serialized commands and odometry from the same run. Observation
age remains a separate candidate; do not mix raw observation age with clock
handoff. Keep original failed 937, earlier derived Stop 388 missing provenance,
and all rejected architecture outcomes. Compare the new sealed world before
another formulation change. No safety margin, tolerance, weights, timeout,
solver, model or actuator limit change.

If demonstrated, transfer the actually published source clock on Bundle-to-exact
handoff and remove the obsolete same-identity clock mask. Preserve ordinary
same-plan continuous publication and stale-decision rejection. Verify native
before/after, package/build, exact replay and fresh single/dev2. No production
edits while builds or simulation run. Seal/register/spec/status/local commit;
then continue all remaining coupled/intents/gates/submission acceptance.
