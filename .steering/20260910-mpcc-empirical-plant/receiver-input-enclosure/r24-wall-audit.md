# Dev2-r24 first wall rejection audit

2026-09-11 JST. Baseline e53e3791, buildr51/testsr41. Full M4–M6 remain open.
The run precedes the corner-enclosure repair described below. The prior review rejection is resolved;
no user approval is pending.

First active failure D1decision947/source245 at1.60m/s/WP31,
wall1789076390.272017471, now10.129999773/control10.259999773. Previous actual
publication946/source249 at10.094999774 is separate and remains correctly
marked invalid for cross-association. D2decision990/1.68mps is1.506s later,
after D1Emergency; teardown clock-epoch failures are not the first cause.
Protected result files restored. First D1callback151.007ms, primary131.602,
alternate9.574, Stop successor0.362ms; predecessor946already25.648ms.
Live25ms and run acceptance fail. Aggregate telemetry includes startup/teardown:
D1p9934.258169ms/28adjacentpairs, D2p9934.478895ms/27pairs.

Current r116 exact replay reproduces source245 final steering-unreachable
with underlying AppliedWallRejected6 at10.684999773,23.219ms. The top-level
steering reason masks terminal failure of a current feedback candidate; do
not claim first packet alone was invalid. Previous946/source249 full proof,
production/materialization/join pass9.938ms, rest11.134999774.

Same-world architecture comparison under original hard bounds:
A accepted33.2501ms, Bleft156.439, Cleft61.684, Dleft172.163, Gleft155.765;
right variants solver-reject. Complete-restY accepted13.8848ms, terminal
progress0.745478/u0,20controls. These are nominal results, not complete applied
certificates or live authority. No all-method physical infeasibility claim.
Files: output/20260911-nine-state-dev2-causal-r24/manifest.json, logs, metrics.

Diagnostic r118 invokes all four original terminal arms separately with exact
source loaded. Ranking prefers immediate. Immediate nominal wall passes but
full applied wall rejects10.979999773 (7.834ms). Source-horizon programme
fails nominal wall (2.924ms); normal-path/track nominal walls pass but applied
walls reject10.809999773/10.684999773 (6.542/5.928ms). All reject.

Fresh r119 tests the actual previous249programme materialized and rebound to
current947observation/history. No association is relabeled. Exact remaining
braking packet -3/0.009417254wire still wall-rejects10.969999773,
rest11.179999773. Full production rejects. Retaining the old programme alone
is falsified again for this new world. All64native sampled response paths
remain within the original enclosure, rest and avoid wall (112128scalarchecks).
Finite response samples cannot prove population clearance.

R120 samples132individual poses inside the original first-contact box
(2x2translation endpoints x33yaw values). None touches the wall. R121 uses
outward interval support of every original margin-expanded body corner against
whole original occupied/unknown cell squares,128candidate separating directions.
For the original immediate programme all39rectangle contacts involve one cell,
best direction world -x. First rectangle rejection10.979999773 separates by
0.019678576m, but full support still fails from11.054999773 and reaches
-0.012405141m before rest11.174999773. Therefore rectangle corners alone do
not explain the complete failure. No geometric gate has been changed.

Next: diagnose the remaining numerical position/heading enclosure width;
compare equivalent coordinate frames or existing hybrid partitions on this
sealed world. Do not add margins, change receiver profile, drop input responses
or call a finite sampling success a certificate. Every new experiment and
revisit condition must be added to the registry before closing the slice.

R122world-axis integration worsens the wall bound (support−.110312126m);
rejected. R123corner diagnostic compile failed; r124wrong extra-AABB geometry
rejects209steps; r125intersection still rejects39steps. R126stable trigonometric
difference with exact shared midpoint yaw increment preserves all1744body
components and111616nativecorner samples and proves additional corner/world
AABB separation for every original contact cell through rest. Production and
full gates remain to integrate. See [design](corner-displacement-design.md).

Production integration now passes buildr52/testsr42 and exact full replay947
through materialization/join, while true peer/history failures remain rejected.
[Design and validation](corner-displacement-design.md) record the boundary;
freshdev2-r25 is still required before integrated acceptance.
