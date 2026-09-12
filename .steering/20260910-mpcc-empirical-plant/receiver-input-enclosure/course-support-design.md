# Accepted progress rows and immutable course support

2026-09-12 JST. Baseline/rollback `44a5e551` (production control `6c7c9615`).
Authorized implementation; full M4–M6 completion remains open. This is a map
provenance prerequisite, not promotion of the diagnostic prepared-launch solver.

## Broken invariant and evidence

The immutable course window must contain the coordinates that the accepted
normal solution's exact native replay may query. The controller currently copies
only nominal semantic progress-box bounds and a forward Stop extension.
Accepted virtual-speed residuals may be slightly negative at a declared zero
lower bound. Native replay starts at the exact semantic state and integrates those
unchanged controls; it can query a coordinate just before the copied first knot.
The valid QP then fails the physical adapter before its body/obstacle proof.

R461's four-stage preparation candidate passes the full optimized nine-state
pipeline. R462 replaces the fixed preparation duration with a target derived
from existing global/cumulative steering limits. Its QP solves, but exact replay
fails at substep108. Virtual speed at control stage6 is−1.643128995070276e−6m/s;
the course window starts at the semantic theta0 coordinate. The original strict
1e−9m query allowance correctly rejects the unavailable prior course data.

R464 fixes only that missing immutable static support. The original QP matrix,
bounds, solved primal, controls, state, peer field, timestamps and physical
requirements remain fixed. Forty-five overlapping static XY/heading knots and
the origin anchor match bit for bit in the same run/grid. Only the preceding
waypoint's geometry is used, and its progress is recomputed from the unchanged
origin. Original support reproduces substep108 rejection. Extended support passes
all exact wall/peer/terminal proofs with the same6.132282723880778e−5 original row
violation; final physical velocity0.779527673m/s. R463's stricter diagnostic setup
rejected four cumulative-distance rounding differences of at most7.1e−15m; its
input rejection is preserved, not converted into a solver result.

This is not evidence that original r74source618 can solve with map support alone:
its earlier wall affine problem still needs the separate representation/candidate
work. No prepared steering/acceleration sequence has live authority.

## Change and remaining owners

Add a pure adapter helper that computes static progress support for accepted
state rows and exact integration of accepted virtual-speed rows. It includes the
existing solved-inaccurate multiplier. For finite row bounds L/U and physical
absolute/relative tolerances a/r, the conservative acceptance outset is
`(a + r * max(abs(L), abs(U))) / (1 - r)`, with the existing multiplier applied
to a/r. Singleton virtual-speed rows are supported; nonfinite/malformed inputs
fail closed. The union includes every prefix from the semantic initial progress.

The controller passes this range to its existing bounded course-window builder,
which copies real preceding/following reference waypoints. Preserve the existing
forward Stop extension. Do not extrapolate a course, clamp a solved virtual speed,
move an origin, enlarge a QP bound/tolerance, retry a failed proof or publish a
cached trajectory. Additional static data remains part of the existing immutable
course-window fingerprint. No new authority, phase, grace, timeout or fallback.

Retire the caller's nominal-state-box-only range loop. The model, QP, physical
proof, current history and publisher gates remain their existing owners. The
range is allocation support, not a physical certificate; unavailable actual map
support still rejects.

Capture the actual numerical owner's tolerance once on controller construction
and copy that scalar configuration into tactical snapshots. Those clones
intentionally have no Track/Cruise solver; a new dependency on its live pointer
would incorrectly reject their course windows. Missing captured configuration
still rejects rather than assuming the zero-initialized tolerance default.

The additional data can change boundary finite-difference tangents and increases
copied static support. Validate frozen numerical/physical outcomes and runtime
before calling the slice integrated. Pure singleton/nonfinite/tolerance and
accepted-negative-input regressions cover the new helper; package tests/build,
actual-library replay, source/identity review and a standard dynamic run remain.

## Work

- [x] R461 prepared free-control candidate and R462 bounded-target failure captured.
- [x] R464 same original QP/control proof isolates missing preceding static support.
- [x] Add pure support helper and wire existing real-map window builder.
- [x] R465 focused native course-support regressions: all four pass.
- [x] Build115: 26 packages. Package102: 2596 records, zero failures/errors/skips.
- [x] Source/identity checks: all105 pass. Quality and diff checks pass.
- [x] R469 actual built-library replay: allocation[-0.219778431,20.181305664]m
  fits the same immutable real map. Unchanged original QP/primal rejects at108
  with old data and passes full exact wall/dynamic/terminal proof with prior knot.
- [x] R468 removes all preparation input bindings while retaining the connected
  native preparation tangent, pose-specific wall rows and independent conditioning.
  All original control variables remain free. Full optimized source proof accepts
  in82.220016ms; final speed1.063094829m/s. This diagnostic comparison removes the
  need to promote a repeated fixed preparation phase; its other numerical/wall
  changes still need separate producer validation and current/live acceptance.
- [ ] Standard dynamic validation; seal/local commit.
- [ ] Continue candidate/pose-wall/solver work, current timing, all M4–M6 and final submission/eval.

R460 remains a physical oracle: its adapter call used zero tolerance to construct
true physical input limits, whereas R461/R462 use the current solver's inset
bounds (e.g. steering rate0.714215 instead of physical0.731707rad/s). The oracle
is not proof of the original affine row/cumulative-prefix or current receiver
contract. Its four full physical witnesses cannot be published as a fallback.

R466 preserves the source-test setup failure that selected an earlier call rather
than the producer definition. Definition-specific matching passes before package102.
R467's standalone driver failed to compile on a controller-local namespace alias;
the corrected public namespace is verified under distinct R469. Neither setup
failure is a physical or solver rejection. Payload hashes are in
[course-support-evidence.json](course-support-evidence.json).
