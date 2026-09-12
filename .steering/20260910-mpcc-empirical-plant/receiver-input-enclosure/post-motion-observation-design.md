# First stationary final loss after actual normal motion

2026-09-13. Authorized autonomous M4–M6 work; baseline/rollback `64dac497`.
Controller `fe976b39`, algorithms/model `8d9810e0` before this diagnostic change.

The detection gap is bounded recording: single-r7 final stationary1012 and
force-r3 final stationary1149 have no paired exact selection capture. Earlier
startup/Ready stationary779/787 already consumed the fixed stationary bucket.
Moving candidate rejection is not final loss, and the last normal publication
is not evidence of why its successor failed. Stop/rest/restart remains open.

Preserve all four existing buckets and their original paths. Add exactly one
lifetime bucket for the first stationary final due/active failure after an
actual authenticated normal publication with observed abs(forward speed)>0.1m/s.
This is the existing diagnostic moving classification, not a control threshold.
The original pair is submitted first. A second, separately named pair carries
that earlier publication's decision, raw pose epoch/speed and actual transaction.
Both pairs are immutable; disk I/O remains on the existing recorder worker.
The extra bucket is never replenished, including after clock/session changes.

Capture the motion witness only after matches_after_publication and the actual
published identity succeed. Clear it on nonfinite/backward control clock; reject
future/same-decision, nonfinite, stationary, unsourced or mismatched paired
witnesses before the new bucket is consumed. Merely receiving movement, solving,
selecting a candidate or sending Emergency cannot create this witness. The
record is first post-motion stationary loss, not a claim of terminal stall or
successful restart. It does not claim a new session solely from increasing time.

No authority, model, solver, margins, fingerprints, scheduling, recovery or
normal return path changes. No obsolete normal path to remove. The obsolete
assumption is that startup stationary capture also covers a later loss.
A separate recorder/thread per phase or resetting existing buckets would add
workers or replace evidence; the fifth fixed bucket avoids both.

Acceptance: native old-recorder red/new-recorder green for startup + moving +
later stationary paired preservation, invalid witness rejection and duplicate
bounds; source placement/clock-reset contract; full build122 and package109;
original physical wall/deadline replays. Then commit source and one bounded
standard single-r8 (120s host cap, existing six-lap/600s setup) to obtain missing
post-motion evidence. Preserve controls/JSON/DLL, review actual final authority,
callback timing, public topic frequencies, any later restart. A new live run is
justified by this missing witness, not another unchanged model experiment.
All integrated M4–M6 acceptance remains open until independently demonstrated.

## Verification and review

R583 preserves the initial temporary-directory collision after the expected old
failure. R584 changes only fixture isolation and reuses the exact old/new objects:
old2 fail/new2 pass. The source contract106 passes with pytest plugin autoload
disabled; the two earlier host collection errors from unrelated launch plugins
are preserved. Build122 all26 and package109 all2615/66 pass with no test errors,
failures or skips. R585 preserves the missing read-only steering mount setup failure. R586 freshly compiled driver preserves the two old wall
rejections and the actual before-publication deadline rejection with exact
source/domain/history/prior publication and check/time identities.

Review: the moving witness is assigned after actual dispatch authentication,
never in solver/candidate selection or Emergency publication. Both original
attempts are submitted before adding diagnostic metadata; the separate path
prevents overwriting. No new thread or unbounded reset/retry bucket. Motion
metadata does not enter any physical or authority fingerprint. The bounded
live run will check added callback cost and that this event actually occurs.
[Results and payload hashes](post-motion-observation-evidence.json).
