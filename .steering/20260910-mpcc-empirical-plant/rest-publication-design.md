# Represent the certified temporal interval at complete body rest

Rollback HEAD `d7fffe15`. Dev2-r4 first moving Emergency is D2decision853,
now8.419999811/control8.549999811000001, world8427277602506770818
(`74f3b45471058982`). Preserve all outcomes; M4–M6 are open.

The previous repair does execute the certified Stop inputs: Stop322is
materialized at846for0.225s and used through852. At853the old cursor expires.
Captured852/853Requests both have control-origin u/vy/r exactly0. Source322has
no solver source because it is a derived certified terminal artifact. The
complete physical-plan evidence is present and independently reloads, validates
and replays; do not invent a solver source from the current normal snapshot.

Current-world Stop proof passes both wall and peers and supplies5dense samples
over one25ms interval at rest. Bundle construction nevertheless rejects speed0
as InvalidIdentity. Removing that entry check alone still rejects the artifact:
it demands strictly increasing path distance, while this temporal interval has
zero distance. This is the earliest broken producer/representation contract.
The downstream atomic Stop supervisor correctly remains fail-closed after
losing certified publication; it is not the producer to relax or remove.

Baseline exact replay: `output/20260910-nine-state-rest-publication-diagnostic-r1`.
Entry-only comparison: `output/20260910-nine-state-rest-publication-comparison-r2`
rejects InvalidExecutionArtifact. Complete representation comparison:
`output/20260910-nine-state-rest-publication-comparison-r5` passes artifact,
native rest replay, current-world full-suffix join and production adapter at
speed0/acceleration−3 for both exact852and853Requests. No solves, state/peer
substitution, horizon extension, margin/tolerance change or extra hold rule.
Comparisonr1failed to compile due to a diagnostic include guard collision;
r3/r4had incomplete preparation and do not establish representation acceptance.

Repair: allow nonnegative initial speed; allow equal adjacent path distances
only for terminal-body-rest artifacts whose endpoint u/vy/r are all exactly0,
wire acceleration is nonpositive and virtual progress speed is exactly0.
Decreasing distance and moving/accelerating fake rest stay rejected. Native
full/continuation replay carries the existing stationary-suffix semantics with
zero velocity tolerance. Full wall/peer proof and exact terminal body rest
remain mandatory before publication. A later normal still needs its own fresh
certified authority to restart; no direct launch or low-speed fallback is added.

Tests must exercise exhausted source → current-world complete rest → immutable
temporal artifact → current-world join, reject false rest and peer occupancy.
Then package build/tests, exact852/853replay and fresh dev2. Timing and actual
rest/restart in the integrated simulator remain required.

Production validation: buildr16passes26packages in17.9s; testsr12pass2394records,
0errors/failures/skips. Exact852/853production replay (toolr15) accepts the new
rest artifact, native replay and current-world full suffix at speed0/wire−3.
Baseline852accepted/853cursor-unavailable are unchanged. Replay reloads the
complete original certified plan rather than assigning a synthetic solver source.

Dev2-r4timing includes startup/teardown: D1/D2raw count1422/1380,
p95 12.83838685/16.53721235ms, p99 17.48796019/27.12970131ms,
max25.514559/47.267045ms, overruns1/21, adjacent pairs0/1, overflow0.
Materialization846costs18.590113ms for the whole callback. The timing outliers
need phase attribution; they are not claimed to be caused by materialization.
Command receipt Hz39.98972335/39.99494287, max34.24668312/40.86184502ms;
source-clock duplicates0/1, backward0, invalid commands0, serialization join
rejects0. Evidence: `output/20260910-nine-state-dev2-r4-metrics`.
