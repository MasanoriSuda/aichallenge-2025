# Recurrent direct-policy requirements

## Objective

Build an offline recurrent steering candidate that learns complete
pre-contact-teacher sequences from LiDAR and synchronized ego speed, without
changing production authority.

## Invariants

- Production TinyLidarNet, launch defaults, fixed LiDAR brake and final command
  interface remain frozen.
- Existing generated datasets are immutable inputs; a new dataset schema is
  derived with explicit source identity and speed synchronization evidence.
- Train/validation stay run-disjoint.
- Sequence chunks never cross a run boundary and validation preserves temporal
  order.
- The model predicts steering directly.  It does not inherit a residual gate,
  teacher-only runtime rule or production fallback.
- LiDAR angular samples remain explicit features before the recurrent layer.
- Runtime implementation begins only after offline admission.

## Definition of Done

1. Synchronize `/localization/kinematic_state` speed to every accepted scan
   within an explicit tolerance.
2. Reject missing, stale, non-finite or identity-mismatched source data.
3. Train a per-beam pressure-token + speed-conditioned GRU on full temporal
   chunks.
4. Compare against the frozen base on material, anchor, full-run and unseen-run
   metrics.
5. Keep production unchanged unless every offline gate passes.
