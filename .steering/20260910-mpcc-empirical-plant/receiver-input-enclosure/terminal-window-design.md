# Complete publication interval versus proved-rest horizon

2026-09-12, baseline/rollback5366b176. R57/M4-M6 remain incomplete.

R57D1post1541/index14: first original programme epoch26.504999408;
nominal26.854999408, current26.859999399, before26.864999399, after26.874999399,
original indexed deadline26.879999408. Original proved rest26.864999408000003
falls between before/after. Individual25ms bracket is admissible. Current old
measurement is compatible; current check is ~.047ms, not full repropagation.
R342 reproduces exactsource/currenthash, beforepass/afterfail, validpacketwindow
and originalrestnotcoveringafter. No actualreceipt minted. Actualsend remains
recorded. This is independent of earlierR57D1pre1118/D2pre1086preemptions.

First broken producer invariant: an immutable dispatch candidate can declare an
original possible publication interval extending beyond its own proof horizon.
Checking only current/actual-before clocks permits such an incomplete candidate;
only the after guard detects the uncovered already-issued command. The producer
must require the ENTIRE original indexed publication window to lie within each
selected complete proof (original and independently current, if present).

Fix scope: enforce full-window proof coverage before minting DispatchCandidate.
Retain original rest values, nominal epochs,25ms intervals, actual before/after
checks, receiver250ms, all context/prefix/source/body/world/physical conditions.
Partial terminal-tail intervals produce explicit RestExpired, never a certified
normal candidate. Existing bounded Emergency remains the unavailable-proof
outcome; no positive hold, new normal fallback, retiming, grace or timeout.
Previously sent braking inputs remain in the actual ledger; do not erase them.
No claim of normal availability, rest/restart completion or movingrace acceptance.

Alternative independently extend a complete source proof through another whole
packet interval, or explicitly certify terminal invariant/rest lifecycle, would
need separate proof/ownership and full world coverage. Do not round or relabel a
rest timestamp as that proof. They are unimplemented alternatives.

Failing native regression: first repeated braking-tail interval that crosses the
original rest boundary, with original source-bound earlier transactions. Before
fix it is a candidate despite incomplete declared coverage. After fix it must be
RestExpired before issue. Retain ordinary early-window/domain and actualpre/post
regressions; original failure snapshots should change only candidate eligibility
where wholewindow is incomplete. Build/package/real-library replay, then runtime.

R343 fails on unchanged5366b176libraries: exact source-bound repeated-tail fixture
returnsCandidate despite deadline beyond original rest. The new producer checks
original indexed deadline against original and selected independent complete
proof horizons before minting. Actual after/clock/slew checks remain unchanged.

R344all149native tests pass; buildr97all26 and testsr85all2565records/66groups
(including source105) pass. R345six prior exact-library cases retain domain
adoption/prefix/identity/actualpre-post results. R346R57earliestpre cases retain
exactsource/currentproof and reject at actuallate clocks; terminalpost1541 now
rejects candidate before issue with unchanged originalrest and originalsourcehash.
Its historical already-issued command remains in the saved capture; no current
proof or actualreceipt is minted. [Evidence](terminal-window-evidence.json).
Nextstandardr58; earlierpreemption/coverage and all M4-M6 remain open.
