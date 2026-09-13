# Bounded post-Rejoin reserved-result observation

2026-09-13. Baseline39df1ffb/controlb3613667. Authorized diagnostic work; no push.
R701shows two live Rejoin completions followed by short Cruise pulses/braking and
34late reserved outcomes, but earlier observations consumed the existing buckets.
This missing immutable worker input is the failed observation invariant. The
producer is scheduled_normal_control's reservation-invalid recorder admission.

Add one dedicated recorder and one first-admitted flag for a Cruise reserved result
strictly after an actual certified Rejoin publication and strictly past its original
first deadline. A source/bootstrap with no reserved priors cannot use this slot.
Record the original request/certificate/domain/actual ledger and current observation,
worker/domain wall time and the exact prior Rejoin command/actual transaction.
The witness is set only after matches_after_publication and an opaque receipt,
with programme publication intent Rejoin (not its Stop tail). Reset its epoch on
clock regression; no witness may come from the future of the saved job.

No solver/order/input/model/timer/reservation/authority change. Preserve existing
buckets, the original rejection and actual sends. Write via the existing recorder
worker; one first admission avoids repeatedly copying a large request after the slot
is taken. The new optional YAML metadata is diagnostic only, authority=false; missing
old metadata remains absent. It does not claim current authorization or gear truth.

Validate exact source/current IDs and packet serialization, the separate bounded
recorder after earlier slots are consumed, reset/call-site invariants, source tests,
package/native/build. Then local commit and single-r18as diagnostic. Native replay
of the captured request will compare each original proof branch before any timing/
programme repair. Individual branch timings are to be measured offline; current
runtime metadata contains whole worker and starting-domain timing only. Local
observability acceptance is not race acceptance. Rollback39df1ffb.
[Failure and next comparison](post-rejoin-source-audit.md).

R703passes the single-request writer test. Review found that the existing worker
can try up to four source plans; its total time cannot be attributed to only its
last request. Capture each actually attempted request, result reason/certificate
and per-attempt wall time while the post-Rejoin diagnostic is armed. Preserve
failed attempts too. Move the worker-owned request after evaluation; the live
outcome retains its original copy. The one recorder worker serially writes up to
four leaf snapshots with separate world/grid files and records all paths in the
parent. No recursion beyond one level. Individual Stop-profile branches inside
each attempt still require offline replay. Capture overhead is diagnostic and
whole-worker time includes it; it is not a race timing claim.

R704passes27snapshot+166retained/source-reservation tests=193, including separate
actual Rejoin receipt and inspected Cruise source, old slot consumed/new dedicated
slot accepted exactly once, failed+accepted source leaf snapshots with separate
files and individual timing. Source118passes; build132all26frozen-source pass.
Package119:Summary: 2640 tests, 0 errors, 0 failures, 0 skipped. Review found no further changed-path defect; live capture
and individual source-profile replay remain unverified.
[Validation](post-rejoin-observation-validation.json).
