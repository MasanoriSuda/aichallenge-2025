# Original scheduling and source availability audit

2026-09-12 JST, production baseline ad18d262. Build r99 all 26 packages and
package tests r86 all 2570 records/66 groups belong to this unchanged source.
This slice changes evidence and documentation only. M4–M6 remain incomplete.
[Evidence hashes](scheduling-availability-evidence.json),
[r60 causal chronology and CPU revisit justification](r60-scheduling-revisit.md).

## Runtime results and restored experiment

Standard r60 fails its original final send window. Earliest D2 pre489 is a
lateral/yaw starting-domain miss; D1 post1074 already has an accepted independent
current domain. Its final actual send crosses the original 25 ms deadline while
remaining inside the complete source-rest horizon. R354 reproduces all three
captured source/current identities, retained actual histories and guard outcomes.
The raw diagnostic command publisher precedes the final control publisher; its
completion clock cannot substitute for the final control-command send clock.

R61 repeats the bounded, owned-container CPU partition on unchanged ad18d262,
using the changed short-proof cost/preemption evidence as its revisit condition.
It still fails: first D1 pre5453, source5452/4905/index0, nominal134.714996989,
entry134.734996988, before134.784996987, deadline134.739996989. Current domain
accepted:2.428092 ms wall/1.158538 ms CPU, one involuntary switch. Callback11.412 ms
wall/2.114695 ms CPU; about8 ms within MPC after the measured selection phases
is not localized. Do not infer a competing thread from these aggregate counters.
Later D1 pre10812 also fails; no D2 guard or post-send guard failure, no laps.
Unity's saved log reaches global Start even though per-vehicle states stay Ready.
A missing global start is therefore not the explanation for this run.

The owned runner was interrupted after sufficient failure evidence. All 30
modified owned threads exited; no restoration errors or unrelated-thread changes.
Compose cleanup and protected result/DLL hashes are verified. CPU policy is not
promoted. The run is diagnostic and cannot satisfy standard race acceptance.

## Source availability and bounded architecture comparison

Before the first teardown signals, D1/D2 unique jobs=7519/4254, worker median
32.592/35.418 ms; 7054/4068 exceed one 25 ms period. New source admissions=7518/4253,
actual-prefix rejections=7127/4036, candidate successes=389/215. These are gate
counts, not proof that every rejection has only one cause. The immutable C5 job
currently starts at the next old nominal appointment with zero pending prior
packets. Meanwhile the main callback may publish certified old braking packets;
those actual extra transactions correctly invalidate the new job's prefix.

R355 compares pending prior counts 0/1/2 and immediate versus full source-horizon
programmes on two frozen sources. All six immediate sources and independently
recomputed composite domains/full worlds pass; the six longer sources fail.
R356's reader incorrectly required a compatible context on a Track→Cruise scene
and aborts. R357 preserves that original semantic conflict and correctly rejects
current adoption, including the lead1 candidate. No context is overwritten to
obtain acceptance. Detailed terminal fields still do not localize every outer
long-programme rejection; do not label them physical infeasibility.

R358 instead uses r61 D1 dispatch908/source905, with unchanged Cruise context,
source.now9.924999778 and fresh.now9.999999776. Its two captured actual old packets
match the original programme values, indices, windows and full retained history.
Immediate and existing full source-horizon programmes both pass complete new
source proofs. All six independently computed composite domains/full original
worlds pass. Pending counts 0/1 correctly reject the two extra transactions;
count2 passes the actual current context/prefix/full physical proof and the
original before-send guard, for both programmes. New first9.994999782,
deadline10.019999782. Actual steering step0.008287911631089055 is below unchanged
allowed0.02364893690516259. The longer source has13 positive suffix packets before
braking; immediate has1. No existing proof is retimed, and no replay sends.

This establishes a source-producer mismatch in the same-context scene, not a
live solution. Future-prior source IDs in the diagnostic are taken from frozen
actual transactions and explicitly labelled conditional. Production must freeze
those expected IDs from the already active immutable certificate BEFORE planning.
Pending programmes also require a distinct full-composite domain binding retaining
all original delayed prior inputs; deleting the existing unsupported guard alone
would be unsound. The final source identity, actual-prefix count, context, full
world, slew and every original 25 ms send window must continue to agree.

## Next implementation obligations

- Derive a future appointment and expected prior identities causally from the
  active certificate; retain a ready future job until its original appointment.
  Early, skipped, late, revoked, replaced and different-prefix jobs cannot issue.
- Certify the entire original prior+new programme for the starting-domain theorem.
  Bind private evidence to the exact certificate and suffix index; preserve
  current full proof when domain membership or its supported scope fails.
- Account for the latest permitted actual predecessor send when proving a future
  steering step. Nominal predecessor timing alone is not a universal slew proof.
- Compare source-horizon selection and authority intent through its accelerating
  prefix and braking suffix; a later packet is not necessarily already braking.
- Establish focused failing regressions before source edits, then native/full
  build/package tests, exact replays and standard dev2. Rollback baseline ad18d262.
  All mission, full stop/restart, repeated race/gate and same submission/eval
  acceptance remains open. No threshold, rate, margin, receiver bound or fallback
  change is authorized by this diagnostic result.
