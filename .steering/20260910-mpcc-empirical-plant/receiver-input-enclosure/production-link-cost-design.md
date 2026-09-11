# Actual production link cost comparison, baseline dbbe5fe6

2026-09-12 JST. Authorized M1-M6 continuation. Diagnostic comparison only.
R54 standard dev2 demonstrates current-footprint Recovery demand (fully deferred
Normal windows average about 0.06-0.08 ms). It still fails original publication
windows: D1 decision1157, nominal16.349999644, before16.374999633,
after16.379999633, deadline16.374999644. Current proof16.208323 ms wall /
11.252341 ms CPU; Recovery0.073 ms. Main callback17.774 ms wall/12.093230 CPU.
This event precedes shutdown1789144437.0304444. No race acceptance or laps.

A previously unmeasured implementation distinction: r306's isolated baseline
compiles numerical prediction and retained/applied code directly into its replay
executable, while production uses the built vehicle-model shared library and
retained/applied static library. The replay is about7 ms; production thread CPU
about11 ms. System load/cache frequency also differ, so linkage is a hypothesis,
not an established cause. Compare the same exact R51/R53/R54 inputs under the
same isolated conditions, changing only source vs actual built library linking.
Use unchanged source/physical model, no math flags or bound/profile changes.
Capture every body/corner/rest range and original/current hash; identify any
floating result difference before treating timings as comparable. R54 needs its
actual post-send boundary classification, not r306's pre-send-only assertion.

If library linkage accounts for a material difference, inspect actual symbols/
call paths and a separately scoped compiler/link hypothesis. This grants no
production optimization, no removal of current full proof, and no live timing
acceptance. If not, reject it rather than repeat old SIMD/trig/coarsening variants.
R193/R194 old point-population reuse and timeout/clock relaxation stay rejected.
All M4-M6 remaining work is retained in the current tasklist. Rollback base is
this unchanged production commit; no production edit is part of this comparison.

R316 completed: actual production libraries and direct-source executable have
identical original/current body, rigid-corner and rest output bits on all three
R51/R53/R54inputs. Medians respectively6.76/6.91/6.89ms vs6.77/6.94/6.84ms.
Linkage is rejected as the cause; no compiler/link change is promoted. R54actual
before-pass/after-fail is reproduced with its recorded actual send in a diagnostic
ledger. Original input/current physical fingerprints match.
[Starting-domain comparison](starting-domain-design.md), [evidence](starting-domain-evidence.json).
