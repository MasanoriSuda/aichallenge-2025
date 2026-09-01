# Design

The v11 artifact proved that full steering authority is needed near a blocked
side, but its normal corpus contains only one train sequence.  The current
single-vehicle shadow run exposes repeated clean-track sectors where the model
still emits material corrections, mostly cancelling the frozen base command.

Build a successor immutable normal-anchor corpus from:

- the previous admitted train anchor;
- the current v11 shadow single-vehicle run as an additional train anchor;
- the previous independent validation anchor unchanged.

The shadow run is used instead of the authority run so its trajectory and
command remain owned by the admitted frozen base.  Every accepted LiDAR frame
receives an exact zero correction label.  Training architecture, teacher
corpus, full-range representation and loss parameters remain unchanged; the
only independent variable is current-distribution normal coverage.

Comparison is offline first:

1. strict aggregate/peer/held-out/normal audit;
2. replay on both clean shadow and authority bags;
3. frozen replay on the v10 wall-stall suffix;
4. shadow closed loop only if all preceding evidence improves or remains
   within the existing Gate.

No runtime confidence threshold is introduced because it would hide rather
than correct a learned classification error.
