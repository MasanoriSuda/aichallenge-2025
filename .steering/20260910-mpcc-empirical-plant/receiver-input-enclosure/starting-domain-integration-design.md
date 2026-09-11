# Authenticated starting-domain integration

2026-09-12 JST, baseline/rollback `76d7ecc4`. Scope: sole scheduled dispatcher,
worker numerical evidence, additive failure observations and regression tests.

R51/R53/R54 show full current numerical integration consumes 11 ms CPU in a
25 ms original publication window under bursty simulator clocks. Removing unused
Recovery work did not repair R54. R316 rejects linkage as the cause; R317–R321
compare an independent starting-set theorem and reproduce all three populations.
Their sealed comparisons remain diagnostic evidence, not live acceptance.

Create immutable worker evidence tied to the exact source certificate. Only the
first original unsent packet is supported. The causal set is the hull of original
source samples from source observation through that packet's original deadline;
independently propagate every state and original starting time through full rest.

The dispatcher preserves measurement category, strict component epochs, actual
ledger/prefix, selected context/generation and original payload gates. It computes
an exact new short prefix, requires whole-state/frame/offset/time membership,
then checks the complete independent set against current wall, peers and Follow.
Follow uses begin-time minima only for strictly nondecreasing progress knots and
nonnegative terminal speed. Unsupported or rejected set evidence may use the
existing complete current point proof; it cannot authorize anything by itself.

CurrentDomainProof is distinct from CurrentPhysicalProof/PendingInputTube. Its
private creation binds original source, domain, current prefix and decision.
Original source identity stays in the actual ledger; physical identity is separate.
No publication window, rate, receiver assumption, body constraint, margin, solver
budget, positive hold or Emergency/Recovery policy changes. No old future tube
is relabeled or moved to a new observation. Existing complete point path remains
required for suffix index > 0 and cases outside the new theorem.

Minimum acceptance: source association and immutable-type checks; fresh complete
prefix and all state/time/context/history negatives; full current wall/peer/Follow
failures; unchanged actual pre/post window, steering slew, source and receipt
identity; exact R51/R53/R54 replay; build/package/source tests; then fresh committed
standard dev2 runtime. Worker cost and current cost must both be observed. This
slice cannot complete M4–M6 by unit/replay success alone.

Review correction before promotion: the numerical suffix theorem omits pending
prior packets in a composite source programme. Such packets can affect receiver
inputs even after the new starting state, so prefix membership is insufficient.
Require certificate.first_suffix_index()==0 in the immutable worker factory.
The production producer already has preceding_packet_count=0; general composite
sources retain the full current proof. Test fixtures now explicitly distinguish
this production case from the rejected pending-prior case. No future history is
replaced by nominal prior timestamps. R322/R324 are pre-correction evidence;
R323 failed compilation because the read-only steering mount was missing.

Final native r326:146tests/13suites pass, including six new dispatch cases.
R327 uses actual final production libraries for all three frozen scenes, exact
short/full prefixes, complete independent world proof and unchanged actual
pre/post classification. Buildr94:26packages; testsr82:2560records/66groups;
source105pass. [Sealed evidence](starting-domain-integration-evidence.json).
Next standard committed dev2-r55 tests worker availability and actual deadlines;
no live acceptance is inferred from these isolated results.
