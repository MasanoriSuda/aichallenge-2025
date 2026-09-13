# Current physical interval producer observation

2026-09-13. Baseline cd93cec0; runtime1648f338/single-r24/R789–R792.
The late current physical envelope is rejected before terminal Stop geometry exists.
Its log says valid/feasible but omits preferred containment and the selected interval.
No exact query capture exists at this producer. This is a detection gap, not yet a
proven algorithm defect or physical infeasibility. The outer wall/Stop guard is
required safety, not an obsolete mask to remove.

Capture only the first rejected actual current-envelope query after an authenticated
moving normal send, in the live MPC owner. Record original temporal pose/reference
waypoint, projected lag/lateral/heading and applied lag, scalar bounds, physical
footprint/clearance, sampling step/anchor, all connected clear runs (max32 saved;
overflow explicitly incomplete), selected interval including preferred containment,
same decision/time/intent and prior moving decision/time. Retain the immutable
shared grid, serialize/hash outside control in one owner-held async future. Clones
must not record and failures cannot change the original query or control decision.
No extra geometry scan on the callback; no map/rollout copy, configuration or
solver/authority changes. Diagnostic additions are not a normal candidate.

Meaningful tests save/reload/replay disjoint clear components with a blocked
preferred position, a clear preferred position, negative bounds, immutable map
fingerprint and duplicate/incomplete records. Existing source/native tests, full
build/package and fixed-commit single-r25 must verify timing and capture before
any producer fix. No failing old behavioral test is claimed: the existing behavior
is intentionally preserved and the demonstrated failure is missing evidence.

Added paths: bounded observation type/writer and one call at the producer.
Deleted paths: none; no extra control owner. Retire this one-shot observation after
the captured producer failure has a permanent deterministic regression and supported
fix, or remove it if not reached in the declared experiment. Rollback is the
eventual observation commit; baseline cd93cec0. M4–M6 remain open.

R793 native220/source118 pass; final live-only ordinary-call guard verified by
source118/build140/package127. Build140 all26packages; package127 all2658tests,
zero errors/failures/skips. Removing only declared observation/log additions
recovers the prior controller text exactly. Full precision saved-grid replays
preserve clear/disconnected/invalid cases; overflow remains incomplete and duplicate
write preserves the original. Next fixed-commit single-r25, then exact native replay.
[Validation](current-wall-interval-observation-validation.json).
