# Publication clock results

Baseline 17bcf24b. Real Store native baseline: 32 existing cases pass; the new
same-artifact Bundle-to-exact handoff fails by 0.0044038810139195306 s. Actual
937 cursor is 0.40499999100000039; preserving the 936 publication mapping gives
0.40940387201391992. This reproduces the captured 936/937 Request/ledger mismatch.
Serialized 936 command stamp 9.899999778, wire steering 0.34067103266716003
(physical predecessor for 937 is 0.23740141093730927), acceleration 1.3295962810516357.

Only the causal previous publication anchor changed in exact 937 replay:
terminal still rejects, clearance -0.0014001134308958552 m versus original
-0.0014817573213388169 m. No offline authority or full-cause claim.
936 prefix starts at exact Odometry 9.884999779 but now is 9.899999778 (15 ms);
937 starts at exact Odometry 9.904999778 but now is 9.929999778 (25 ms).
Consumer observation-age alignment is a separate unfinished producer issue.

New sealed world 937 comparisons: all nine A/B/C/D/G normal arms and all four
support Stop arms solver-reject with unchanged production settings/hard bounds.
All-method failure remains Unknown; no physical infeasibility certificate.

Selected repair: on actual published Bundle-to-exact handoff, Store keeps the
clock/cursor association supplied by the accepted final publication even when
artifact identity equals the older executed plan. Uninterrupted same-plan
publication keeps its equivalent original anchor; stale decision and full
identity validation stay unchanged. No new control authority or parameter.
Native after: all 33 Store cases pass (12 ms total). Normal build completes
26 packages in 11.3 s; stderr is existing setuptools deprecation only.
MPCC 60 CTest groups / 2373 records have 0 errors, failures or skips (25.1 s).
Fresh single-r1: six laps in 251.99359130859375 s, penalty 0, no observed moving
Emergency/Recovery or causal-observation warnings. 256 callback windows / 10368
cycles, maximum 13.748 ms, zero overruns. 10369 command receipts at 39.999884 Hz,
maximum gap 34.587145 ms. Odometry 12962 messages / 50.000709 Hz / maximum
42.816162 ms; clock 51847 / 200.006171 Hz / maximum 19.300461 ms. None of these
three topics has a receipt gap over 50 ms, source duplicate/backward or source
gap over 50 ms in this run. This does not erase earlier delivery pauses.
Dev2-r1 is rejected: first moving D1 Emergency 930 at 1.3035718076716887 m/s,
wall time 1788926322.443622415. No finish/details. D1/D2 callback maxima
28.673 / 27.375 ms, 1 / 3 overruns; all reported overrun windows are after the
first D1 Emergency and before explicit teardown, so they do not establish its
cause. One causal-observation warning is D1 startup (spawned, decision 542,
zero speed). No active causal-observation warning is observed.

D1/D2 command receipts 39.993960 / 40.006906 Hz, maxima 50.877094 / 35.742760 ms,
1 / 0 gaps over 50 ms. Both source streams have 3 duplicate/backward stamps;
source gaps over 50 ms are 2 / 1. Odometry receipt maxima 95.826387 / 100.583076 ms,
clock 96.961260 / 98.612309 ms. These source/receipt issues remain separate.
Both campaigns stopped all containers and restored protected user JSONs.

First terminal boundary 929 (world 4007565303905288077) preserves same artifact
376 and preceding Accepted 928. Zero-solve replay reproduces 928 terminal
+0.01128045374819564 m / independent Stop +0.021159567439673843 m; 929 terminal
-0.0007831556743211898 m / independent Stop still accepted +0.002328087195587747 m.
The 928/929 clock anchors are equivalent: source cursor advances 29.999999 ms
with observation time. Replacing only the anchor preserves the same 929 result.
This validates the handoff in this runtime boundary, not all-cycle clock claims.

Unlike the earlier 936/937 pair, peer-only cross-substitution flips this new
pair: 928 with the later peer field rejects (-0.00023400897409575627 m);
929 with the prior peer field accepts (+0.011806859326646713 m). Each field is
projected to the unchanged ego clock. This isolates sensitivity to peer update;
it does not yet establish a faulty peer producer or a feasible live transition.

929 builds and adopts derived Stop 385 (joined, certified-stop). Final 930
inspects 385, with cursor 0.020 s and original publication origin 9.94999978.
The actual-publication observer reports missing solver source, and the inspected
artifact is consequently not serialized. Exact 930 Request is present; prior
ordinary Accepted 928/376 is correctly invalid as a same-385 predecessor.
Do not invent 385 by re-solving 376. Its proof/artifact provenance and actual
multi-tick Stop failure remain open, as did derived 388 in the older b19 run.

Next: peer update/source/receipt epoch audit on exact 928/929, recover derived
Stop evidence, and audit causal alignment of ego pose/speed/steering clocks.
Full coupled/intents/dev3/dev4/gates/submission acceptance remains incomplete.

523 artifacts are SHA256-sealed. Strict registry validates 97 snapshots / 215
experiments. Helper AST, touched-document links and whitespace checks pass.
Protected user JSON hashes match original inputs. Local repair commit follows;
no push or complete-race claim.
