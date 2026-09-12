# Final scheduled authority-loss observation

2026-09-12 JST. Baseline/rollback f06938a6; production1241e0cf.
Authorized autonomous completion. This slice changes diagnostic ownership only.

Standard single-r3 fails D1Ready954 at0.53m/s, waypoint31. The current proof
rejects physical wall6 for source469/job950/index1 after5.142ms wall/5.123ms CPU.
No Start or lap acceptance. Intermediate admission891 already owns the moving
slot, so the final954 source/current/actual-prefix snapshot is absent. R75D2final
2274 had the same detection gap. Neither intermediate snapshot substitutes for
its later final loss. Wall cause beyond this telemetry remains unknown.

The producer is scheduled_normal_control: separate due and already published
active attempts, and transition to Emergency only after both fail. Freeze both
attempts at that final branch, before active reset. Two independent phase
recorders each reserve a fixed stationary/moving slot for an atomic pair of
observations. Existing intermediate/proof/context records cannot consume them.
All serialization and filesystem I/O remain on the recorder worker. Missing
programme/current/context is explicit; only an actually executed CurrentCheck
is marked observed. Copy the real latest ledger transaction independently of
the inspected source. Record generation validity at capture separately from
validity when the asynchronous writer eventually runs.

The final publisher also retains the actual accepted CurrentCheck/domain result
that minted its pending DispatchCandidate; earlier generic final-guard captures
had a default check vector and must not be treated as an observed result.
No replacement normal authority, retry, timing/physical tolerance change, stale
source adoption or trajectory modification is introduced. Existing Emergency,
source selection and strict pre/post-publication checks keep their decisions.

Acceptance: regression rejects the old premature-recording producer; native
recorder test preserves missing due and full active request after an intermediate
slot is consumed, records actual source independently, drains both observations,
and rejects inconsistent pairs/later replacements. Then source tests, all-package
build118, package105, and a fresh standard single-r4 verify the live final join.
A diagnostic slice passing does not establish wall repair or M4–M6 acceptance.

Build118 completed all26 packages. Package105 passed2600 records with zero
errors, failures and skips, including the paired recorder regression. Source
contracts106 pass. The old producer fails the strengthened final-branch check.
Review confirms identical selection order, current proof arguments and Emergency
branch; recording only follows failure and never grants authority. CurrentCheck
is bound to the pending candidate at the producer. The next fresh committed
single-r4 must verify actual final capture and original dynamic acceptance.
[Sealed run and validation evidence](final-selection-observation-evidence.json).
