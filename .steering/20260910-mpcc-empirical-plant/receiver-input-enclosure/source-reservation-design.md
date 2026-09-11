# Causal source reservation and programme availability

2026-09-12 JST. Baseline/rollback4b1920ee. All original model delays, physical
bounds, margins, solver/rate budgets, individual25ms windows and final guards
remain unchanged. No new normal authority or positive fallback is introduced.
[Same-context failing replay](scheduling-availability-audit.md),
[validated composite-input prerequisite](composite-prior-proof-design.md).

## Earliest invariant and chosen structure

R61 D1 source905/dispatch908 has an unchanged Cruise context and two actual old
source packets issued during worker calculation. The original next-slot producer
assumes no such prior inputs and correctly fails actual-prefix adoption. R358
shows that pending2 new immediate/long programmes can satisfy the original current
world/history/clock gate. R362 authenticates their complete composite domains and
current membership. R355's Track→Cruise scene is a separate semantic rejection.
The recorded source worker median exceeds one25ms cycle; immediate programmes
have only one positive packet before braking. A slower source producer cannot
provide continuous positive control by rebuilding that one packet every tick.

SourceReservation privately freezes the already published source certificate,
current generation/ledger snapshot, two upcoming original prior appointments and
ALL five components of each expected source identity before the job starts. It
requires the preceding actual source/index/wire/bracket to match and the complete
pending prior windows to lie within the original old source rest. Its status
checks the exact partial actual prefix: early completion waits for the original
new first epoch; skipped, wrong, extra, reset, revoked or late prefixes invalidate.
Neither a reservation nor waiting state permits a send. Every actual old/new
packet still goes through the sole fresh C5 dispatcher and original before/after
guards. No future transaction can retroactively supply an expected source ID.

Two prior slots are the bounded architecture exercised in r358, not a widened
send window or an established guarantee for every worker duration. Bootstrap and
sources lacking that proved prior horizon retain the original zero-prior source
construction. The new helper grants no authority to hypothetical old inputs;
if those inputs are not actually sent under their existing gates, new adoption
fails. No whole-race success is inferred from this scheduling choice.

## Lifecycle and programme semantics

Only one unadopted job/reservation is produced at a time. The worker mailbox
tracks in-flight work and a scope guard clears it on completion/exception. Main
retains a completed future candidate until its original appointment and validates
partial actual prefix progress; it does not replace that candidate every tick
with a later job. Invalid progress is logged and captured with raw observation,
original request/cursor and the frozen expected prior IDs. Worker/source failure
continues through the existing proof rejection path. Source generation and full
context/geometry/world checks remain mandatory at actual adoption.

Reserved scheduled sources prefer the EXISTING full-source-horizon common programme,
including its original hard velocity ceiling, full nominal/source proof and brake
to rest. The existing fully proved immediate programme remains an alternative
when the long candidate fails. Zero-prior bootstrap jobs and ordinary unscheduled APIs keep their prior order.
The source-horizon normal prefix retains its normal intent until the certified
brake-tail word; index>0 alone no longer labels an accelerating packet Stop.
Current requested Stop/Hold positive-command rejection remains unchanged.

Optional expected_prior_sources and includes_pending_prior fields extend only
the diagnostic v1 snapshot. Existing source/current/ledger fields and evaluator,
participant topic and package contracts retain their names and meanings.

## Verification and remaining work

- R364 three reservation tests pass: IDs frozen before sends; original timing;
  missing/wrong/extra/late/reset/generation and old proof-horizon rejection.
- R365 exposes a new test fixture incorrectly using the final brake word at
  every original prior index even for a long programme. Production correctly
  rejects it. R366 uses the actual indexed planned words and all157 native tests
  pass, including the new long-prefix/terminal intent test.
- Final native367 all158, build102 all26 and package89 all2579/66 pass.
  R368 six final production source arms; same-Cruise pending2 passes private
  composite-domain/current/before gate with13 positive packets, current1.818001ms.
  R369 five old zero-prior source/current identities, exact prefixes and original
  deadline rejections remain. Snapshot exact expected-ID roundtrip passes.
  Standard dev2-r62 and broader mission/Store/runtime acceptance remain pending.
- All M4–M6 runtime, actual Stop/rest/restart, all intents, Recovery/Rejoin/Boost,
  repeated same-finalHEAD races/gates and same submission/eval remain open.

Pre-live review: build101 compiled the first connection (all26) but its blanket
long preference would add worker cost to zero-prior bootstrap jobs, which cannot
survive any intervening actual send. Restrict the new preference to jobs with
reserved prior packets; preserve the old rear-peer ordering for other jobs.
R362 directly measures long source+domain~31ms versus immediate~17ms. Native
regression now compares the same long-capable source with pending2 and pending0.
Build101 is intermediate; incremental build102/tests88 and standard run await
this reviewed final policy. Optional snapshot IDs get an exact64-bit roundtrip
test that keeps declared identities distinct from conflicting actual history.

Build102 all26 and native367 all158 pass. Package88 found four stale source-text
assertions (plus their failing CTest group), not native physical failures: immediate
`ready` adoption, empty expected-prior IDs and blanket index>0 Stop classification.
Update those existing deletion/authority gates to the due-after-reservation edge,
explicit expected IDs and the native-tested programme phase. Preserve every
retired-authority exclusion and committed-source check. Package89 is required;
no failed test is counted as acceptance. Production binaries remain build102.

Final verification: package89 passes after the four existing structural checks
were updated; source contracts105 remain active. Review and intermediate failures
are preserved in [evidence](source-reservation-evidence.json). No full-run
acceptance is inferred from native, build, package or frozen-input replay.
