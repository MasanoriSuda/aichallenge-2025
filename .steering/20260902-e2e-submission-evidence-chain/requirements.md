# Requirements

## Objective

Close the remaining submission-evidence chain gaps after `d666b934` without
changing the qualified controller, model artifacts, launch defaults, or
driving parameters.

## Required guarantees

- Submission readiness must not trust a competition report that was generated
  without the complete production runtime contract.
- The raw and spatial artifacts loaded from the install tree must match both
  the frozen SHA-256 values and their source-tree counterparts.
- A candidate using spatial authority must prove that the spatial model ran
  throughout the evaluated domain with at least 99% coverage, no inference
  errors, no stale intervals, and non-zero authority application.
- Single-vehicle and mixed-peer evidence remain separate.  Missing or failed
  mixed-peer spatial evidence must never promote a candidate to multi-vehicle.
- Historical evidence remains immutable; validation output is written to a
  temporary directory or a new run directory.

## Non-goals

- No controller, model, launch default, ROS interface, speed, acceleration,
  braking, or steering change.
- No retraining or checkpoint replacement.
- No relaxation of existing competition, motion, or spatial Gates.
- No claim that the currently failed mixed-peer run is qualified.

## Definition of Done

- Readiness independently validates the complete expected runtime contract and
  both artifact identities.
- The spatial analyzer supports the E2E student's domain in a mixed-domain run.
- Readiness rejects lax competition reports, wrong source/install artifacts,
  missing spatial evidence, insufficient coverage, stale/error intervals, and
  authority that was never applied.
- The video checklist hashes the install artifacts actually used at runtime and
  separately proves source/install equality.
- Focused and complete TinyLidarNet tests pass.
- Frozen evidence replay preserves the current single-only classification for
  the correct reason.
