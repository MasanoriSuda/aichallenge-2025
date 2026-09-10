# Preserve prediction geometry while batching visualization

Baseline/rollback: `08322c3b`. Run `20260910-nine-state-dev2-r7`, D2decision1154
at1789029342.886923582: callback32.823264ms, prediction marker16.559ms/924points.
The adjacent1155callback34.554804ms has marker0 and Recovery safety16.816ms;
marker construction/publication is a measured contributor, not the entire cause.
R7 is a bounded timing diagnostic, not six-lap acceptance. R6 produced no usable
clock/odometry after Spawned; its stopped simulator and logs remain preserved.

The producer copies a complete ROS Marker per point and publishes the array
to two visualization topics at the existing4Hz cadence. The marker payload has
no control authority. No downstream diagnostic suppression masks this cost.
Compare original per-point SPHERE, reserved per-point SPHERE, and SPHERE_LIST
with identical synthetic924point geometry and ROS C++ serialization. The fixture
uses the observed count, not unrecorded runtime marker coordinates. Measure
construction plus two serializations, separately from integrated callback time.

Replace only this producer after the comparison: one SPHERE_LIST containing
all coordinates in their original order, same map frame,0.5m scales and RGBA.
Identity pose replaces the old zero quaternion (intended unrotated map points).
Each MarkerArray contains DELETEALL then the complete list. Both topics belong
to this controller in the active launch; this clears former per-point IDs,
shrinking trajectories and missed previous updates. Document topic ownership
and migration for any custom subscriber using individual marker IDs first.
Retire the per-point production loop; retain publication cadence and point count.

Validation: native serialization comparison; geometry/style and stale-marker
replacement regression; package build/tests; fresh dev2 timing/race run. Accept
the visualization change only with exact geometry and smaller payload. Integrated
40Hz/25ms acceptance additionally requires resolving any remaining continuous
overruns and stale execution. No proof, solver, rate or safety changes are allowed
by this visualization slice. All M4–M6 checks remain open until their own results.

Native prototype comparison r1: original/reserved arrays214373bytes per topic,
SPHERE_LIST22637bytes; median6.61635/6.55730/0.108935ms (200samples,20warmups).
Production also gives the DELETEALL marker a map frame; the final r2comparison
calls the actual production builder directly. Both are retained as distinct runs.
R19build:26packages/6.17s; r15tests:2397records/62CTestgroups,0errors/failures/skips.
R18/r14 also passed; the final rebuild removes explicit-constructor warnings in
the added test, without changing control source. Existing setuptools deprecation
warnings remain. Native subscriber regression verifies serialized geometry/style,
replacement of924legacy IDs, mismatched coordinate lengths, shrinking and empty
lists. R7aggregate D2p95/p99=18.600853/27.550843ms,1adjacent pair,overflow0;
no invalid command or actuation join rejection. R7remains a timing failure.
