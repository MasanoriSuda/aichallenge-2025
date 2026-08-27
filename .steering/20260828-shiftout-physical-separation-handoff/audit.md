# Audit

## Observed failure

Evidence: `output/20260828-041315/d1/autoware.log`.

- A side=+1 Mission was admitted with `shift=7 m`, `total=25 m`, and
  `goal_ey=2.20 m`.
- ShiftOut remained active at `phase_traveled=32.92 m`, while measured
  `e_y=1.57 m` and the target was laterally on the opposite side of ego.
- The live wall corridor could no longer contain the frozen target goal.
- Reference-only Mission viability preserved the phase until current-world
  canonical proof withdrawal caused Emergency Stop.
- Remaining lateral motion then ended in `actual footprint wall margin
  violated` and Recovery.

## Classification

The first demonstrated defect is a persistent-Mission lifecycle defect:
configuration A retained a path-sample completion condition beyond the
certificate's useful domain.  A stateless current-world bundle would not
retain that obsolete goal as a phase boundary.  This Slice repairs the shared
phase semantic before changing candidate generation or solver formulation.

## Non-causes

- No evidence justifies changing wall margin or target clearance.
- The initial right-side exact snapshot was correctly rejected and was not the
  later selected left-side Mission.
- The wall abort is not the first causal event; it occurs after the stale
  ShiftOut lifecycle has already overrun both planned distances.
