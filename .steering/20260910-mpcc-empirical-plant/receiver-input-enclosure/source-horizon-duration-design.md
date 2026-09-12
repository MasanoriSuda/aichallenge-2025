# Source programme duration ownership

2026-09-12. Baseline / rollback: 25218a0a. The authorized autonomous completion
work includes this producer repair, tests, replay, build and fresh simulation.
No additional authority or publication-time exception is introduced.

The original source-horizon candidate derives its positive packet count from
`continuation_trajectory.elapsed_time_sec.back()`. That trajectory describes a
replay of a different, old changing-input suffix. `build_continuation` can retain
only its independently valid publisher interval after a later failure. Its
shortened proof scope is not the original source schedule's remaining duration.
Using that length for a new constant-input programme prevents constructing the
long candidate before its own physical proof is attempted. The subsequent
one-positive-packet-plus-braking candidate masks this missing proposal.

R385 reproduces this producer edge in frozen r62 D2 pre1104 and D1 post2306:
valid velocity ceilings, 20 original stages, cursor 3 / 4, but six continuation
samples over 30ms become one interval and InvalidArtifact. R387 instead derives
the candidate duration from the same original remaining control stages and
elapsed fraction. D1 post2306 then passes the unchanged complete source proof
with 57 positive packets; its new source fingerprint is intentionally distinct.
D1 pre894 remains identical with 24. D2 still rejects: R390 identifies the
terminal wall check (peer clearance positive), not a missing proposal. R388
was a diagnostic compile failure caused by a missing read-only steering mount.
No diagnostic candidate grants runtime authority.

R389 provides a small native fixture using the existing source-horizon request.
Move only the current pose to y=0.998 and reconstruct its causal prefix: the
old suffix retains PublisherIntervalPrefix and the producer returns a short
programme. At y=0.99 the full suffix remains usable; at y=0.9995 even the first
interval rejects. The regression will require a separately proved long programme
for the middle case and retain rejection for the last case.

Change the proposal duration to the original source schedule's remaining time,
computed from its validated cursor and immutable control-stage durations. Bound
conversion/allocation by the command container's representable capacity; do not
borrow the unrelated continuation sample count. The original current prefix,
complete new programme to body rest, physical velocity ceiling, wall/peer checks,
actual prior history, generation and before/after 25ms gates remain mandatory.
The old suffix proof keeps its accurately limited scope. Bootstrap candidate
ordering remains unchanged. No solver budget, margin, rate, delay or grace changes.

R391 fails at y=.998 before the change. R392 passes all 159 C5 tests afterward.
Build105 passes 26 packages; test92 passes 2,583 records / 66 groups, source105.
R393 produces the expected 24 / 1 / 57 / 33 positive packets in four frozen
scenes. R395 checks nine unchanged source/history/late outcomes and the changed
D1 post2306 source's rejection of old actual suffix history. [Evidence](source-horizon-duration-evidence.json).
R394 incorrectly assumed a later dispatch prefix check had run; R395 confirms
the earlier InvalidPacket rejection and independently checks ActualPrefixMismatch.
Local commit and standard dev2-r64 remain next. Separate current
body-domain coverage, scheduling and all M4-M6 acceptance remain open.

The standard 25218a0a dev2-r63 was stopped after a bounded stationary capture:
no completed laps, three D2 before-publication rejections and no observed
post-publication violation. D2 first1818 requires a complete current point proof
because the body population is outside the proposed domain; live 17.781811ms
wall / 10.675282ms CPU plus late entry crosses the original window. R386
reproduces the source/current fingerprint, actual prefix history and rejection.
The saved boundary current request is decision1818; saved `current_check` and
`domain_use` metadata reflect later entry state, so use the contemporaneous
admission log and exact replay for those labels. No overall acceptance is claimed.
