# Post-Rejoin source and reservation audit

2026-09-13. Continue authorized autonomous M1–M6; local commits, no push.
Current production b3613667; numerical fdee0b8f and physical model8d9810e0 unchanged.

## What the new live run establishes

single-r17on build131/package118 publishes certified Rejoin with original source
and current dispatch IDs intact. Three actual sends join selected complete authority
traces and the actual publication log; this is a lower bound, not a total packet count.
Two reason=rejoin_complete transitions return to Normal and subsequent Cruise sends
1207and1714are observed. The handoff regression is locally fixed and live-positive.

Whole120srun:4296callback samples,max26.410175ms at1009;856normal sends,last1772.
Before-send refusal863keeps original deadline9.849999826; raw before9.854999779.
No actual post-send window violation, no Start/lap. Original DLL/user JSON restored.
No gear topics in MCAP; do not claim gear-state verification from their absence.

## Next earliest unresolved boundary

After both Rejoin completions, the new Cruise source repeatedly publishes a single
positive packet followed by braking. Reserved successor jobs arrive after their
immutable appointment window: job1207takes147.670ms and is rejected at dispatch1213,
now18.914999577 versus first18.829999581/deadline18.854999581. R701captures34such
late reserved results in two specified post-Rejoin decision ranges. This is an
observed timing failure, not proof that any particular solver/physical branch caused it.

At1773a new Cruise1220/job1772is still currently certified, but the detector confirms
1.52sstationary without corroborating contact and RecoveryHoldStop overrides it.
1774sees the resulting normal context invalidation. FinalSafeStop has unknown maneuver
direction with course guard rejection and no observed footprint contact. That is
later than the pulse/brake and reserved successor loss; do not bypass its guards.

Hypothesis: constant-first-word source-horizon programmes become difficult in the
faster Cruise world, while the one-word bootstrap certifies; reserved proofs consume
their entire window. Current evaluate_stop_candidates tries long constant source,
then an intermediate reserved span and short programme. SourceReservation reserves
two certified appointments. Rejoin/Cruise have different bounds/context; do not
compare outcomes as the same immutable world or tune a duration to fit.

The first post-Rejoin worker input/result is not fully captured because earlier
failures consumed existing buckets. Next is a bounded observation-only capture of
that reserved Cruise result after authenticated Rejoin publication. Preserve actual
source/worker/control IDs, original request/forecast/programme, current raw ledger,
individual proof outcomes and timing. Obtain native replay and a sealed comparison
of programme/scheduling alternatives before production changes. No new speed cap,
retry/grace/margin/tolerance or enlarged reservation window as a standalone repair.

r80D2actual post-send deadline, clock-grid hypothesis, physical observation/model,
source walls and all remaining M4–M6 remain open. Full race acceptance is unclaimed.
[Evidence](post-rejoin-source-evidence.json), [handoff fix](recovery-rejoin-handoff-design.md).
