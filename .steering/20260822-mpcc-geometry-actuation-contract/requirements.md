# Slice 2b: canonical progress geometry and actuation contract

## Baseline

- Branch: `develop_july`
- Baseline commit: `f802567`
- Parent slice: `.steering/20260822-mpcc-track-cruise-shadow/`
- Preserved unrelated change: `aichallenge/result-summary.json`

## Root causes repaired

1. Five-state dynamics normalize a zero-length circular-seam stage, while the problem fingerprint
   and physical certificate retain the raw zero distance. A solved problem is therefore certified
   against a different geometry identity.
2. `convert_extended_solution_to_legacy()` places stage-1 predicted velocity in a legacy target-speed
   slot and discards the optimized acceleration input. Shadow telemetry consequently compares two
   different concepts, and future live execution would recompute acceleration outside the MPCC.

## Scope

- Derive one effective progress-stage geometry from waypoint transitions and the exact stage
  distances consumed by temporal dynamics.
- Use that geometry for the five-state problem fingerprint, warm-start identity and physical
  certificate path distances.
- Define a typed MPCC actuation proposal containing stage-1 predicted speed, optimized acceleration,
  curvature and virtual progress speed.
- Keep legacy-layout conversion for prediction-only consumers, but stop calling its speed slot a
  canonical command contract.
- Join the shadow proposal with the final post-arbitration command by decision ID and report
  speed/acceleration/steering deltas with explicit names.

## Non-scope

- No Track/Cruise authority promotion.
- No control parameter, weight, bound, clearance or runtime YAML change.
- No removal or insertion of runtime reference-path waypoints.
- No suppression of real hard-wall or swept-path rejection.
- No change to Emergency or Recovery authority.

## Acceptance

- A deterministic test reproduces raw zero seam distance versus effective progress distance and
  proves the canonical fingerprint/cumulative distances use the effective geometry.
- A deterministic test proves MPCC acceleration is preserved separately from predicted speed and
  rejects malformed/non-finite primal vectors.
- Five-state shadow and live five-state contracts use the same effective geometry source.
- Physical certification receives the same cumulative distances represented in the five-state
  problem fingerprint.
- Shadow remains `selected=0`; production command remains unchanged.
- Build and all package tests pass.
- Repeated single-car simulation reaches at least 99% physical certification or leaves only
  explicitly classified real wall/swept-path rejections; no `solution heading unavailable` remains.

## Rollback

Rollback commit: `f802567`.

Rollback if legacy production command values change outside an already-live five-state seam
certificate, problem identity becomes incomplete, or shadow proposal data reaches command output.
