# Retire the answered current interval observation

2026-09-13, baseline e8ec1e2c/control7ecd0b36, after single-r30/r31.
R832 andR841 exactly reproduce the original clear-run calculation, selected
interval, query, map and physical/normal footprint checks. The original boundary
guards are correct. No interval producer repair or new authority is justified.

Remove only the temporary interval header/writer, CMake source, node include,
one-shot record block and its two members. Delete its five recorder-specific
tests, while preserving every unrelated architecture/Recovery test. Keep old
v1/v2 artifact semantics documented as historical, with no new runtime writer.
Replace measurement coverage with a permanent original1263 regression using
the existing same-map crop: exact146poses/run/guarded interval; current physical
and expanded body clear, +1mm expanded body hits466554. Preserve original1mm
guard/0.2m lateral clearance and Unknown cells outside the fixture crop.

This is removal of an observation, not a behavioral fix: before/after failure
is inapplicable. Establish node byte-equivalence after removing exactly the
observer block/include/members, native semantic regression and unchanged
original source118 tests, full build/package checks. Continue live validation
when the current reservation-admission investigation justifies the next run.
Do not relabel historical r31 as a run of the newly built source. Review final
diff/links/quality and local commit. Original normal/Recovery/proof contracts,
source parameters, clock and publication guards remain unchanged.

Rollback: restore the observer from7f263cc2 only for a new documented evidence
gap. It must not persist merely because removal needs another validation step.
[Closed measurement](render-cap-live-audit.md).
