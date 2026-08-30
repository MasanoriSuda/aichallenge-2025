# Requirements

## Objective

Eliminate the proven command-identity defect in current-world Stop successor
reification.  Preserve the serialized maximum-braking command while modelling
the plant's zero-speed acceleration saturation separately.

## Root cause evidence

- Baseline: `a3383d74 audit(mpcc): classify stop successor reification rejection`
- Run: `output/20260830-184601/d1/autoware.log`
- Decisions 1741 and 1772 had accepted physical Stop proof but bundle rejection
  `command-changed-within-interval` with exactly `3.0 m/s2` acceleration
  difference.
- The producer changed `-3 m/s2` to `0` after the nonlinear velocity entered
  its zero-speed tolerance, without changing `command_interval_index`.

## Constraints

- Do not relax bundle command identity, wall, dynamic or exact proof.
- Do not average commands or clamp the diagnostic result.
- No parameter, tolerance, clearance, solver, Mission, lease, grace, timeout
  or fallback change.
- Keep one canonical normal publisher and all existing fail-closed paths.

## Exit criteria

- Dense Stop provenance distinguishes commanded and effective acceleration.
- Every sample in one serialized interval carries one unchanged commanded
  acceleration.
- Zero-speed saturation is still used only by nonlinear state propagation.
- A Stop that crosses the low-speed tolerance within an interval reifies as a
  valid certified artifact.
- Static tests and `make dev2` show no live
  `command-changed-within-interval` rejection.
