# Track/Cruise canonical wall-bound contract

## Baseline

- Branch: `develop_july`
- Baseline commit: `ed5cc87`
- Preserve the unrelated user change `aichallenge/result-summary.json`.

## Observed phenomenon

`output/20260822-164756` produced a five-state Track/Cruise solution on every eligible cycle, but
54 candidate trajectories failed the exact oriented-footprint wall certificate. Most failures were
at stages 0--2 near waypoints 125--128 and 258--261. The reported scalar lateral-bound reserve was
still about 0.7--1.4 m.

The same run separately identified 73 cycles where the legacy production pose was already in hard
wall contact and three genuine current-to-horizon swept failures. Those are not candidate hard-contact
failures and remain outside this slice.

## Initial root-cause hypothesis

The scalar lateral bounds and the exact footprint certificate do not currently implement the same
physical contract:

- `ReferencePath::update_simple_path_constraints()` subtracts `BicycleModel::safety_margin` from
  the raw path bounds;
- `safety_margin` is `width / sqrt(2) * safety_margin_scale`;
- both live configurations set `mpc.safety_margin_scale: 0.0`;
- comments in the five-state builder nevertheless describe its lateral bounds as already
  footprint-expanded;
- the post-solve certificate evaluates the configured 2.0 m x 1.45 m oriented footprint.

Therefore the QP currently proves only that the vehicle centre is inside the path bounds, while the
certificate asks whether the complete oriented body is inside the wall map. A positive centre-point
reserve is not evidence of body clearance. This remained a real contract limitation, but the
temporary full-margin A/B falsified it as the primary cause of the observed discrete candidate
contacts.

## Confirmed root cause

The five-state QP optimizes an absolute progress state. The physical certificate reconstructed each
solved lateral/heading state at the fixed nominal `ref_wp_id + stage` frame instead of at the solved
progress. Around the failing curves, that nominal frame was 1.0--1.9 m ahead of the solver state.
Consequently the QP and certificate attached one state vector to different course frames.

The canonical contract is:

- the five-state solved progress owns the course-frame position and heading used by physical proof;
- the effective stage geometry supplies explicit, finite, strictly ordered progress-to-course-frame
  provenance;
- missing or out-of-window provenance rejects the certificate instead of silently returning to a
  nominal waypoint;
- the same accepted solver tolerance is used only at provenance-window endpoints;
- the exact oriented-footprint and swept-current-to-horizon proof remains mandatory.

## Required investigation

1. Run an authority-neutral A/B with the established full path margin restored. Do not select the
   shadow command.
2. Compare solve coverage, physical candidate hard contacts, current-pose contacts, swept failures,
   callback timing and legacy lap stability.
3. Reject the hypothesis if candidate hard contacts do not materially decrease or if the failures
   move to unrelated solver/constraint categories.
4. Do not retain a config-only tuning change as the architectural fix. If the hypothesis is
   supported, make the canonical MPCC wall-bound contract explicit and independent from the legacy
   aggressive centre-path setting.
5. Measure nominal-stage progress against solved progress and use the result to choose the canonical
   physical-proof frame.

## Non-scope

- No Track/Cruise authority promotion.
- No production command change during the A/B.
- No clearance weakening and no removal of the exact physical certificate.
- No new post-solve retry or heading-linearized hard rows; those variants were already rejected by
  `.steering/20260822-track-cruise-footprint-bounds/validation.md`.
- No Recovery or overtake performance tuning.

## Exit gate

- The root hypothesis is supported or falsified by a repeatable shadow run.
- Every bound passed to canonical MPCC names whether it is raw centre-path or body-envelope safe.
- Candidate-created physical wall rejects are eliminated or reduced to a separately identified
  reachability/geometry category.
- `authority=shadow, selected=0` remains true.
- Build and package tests pass.
