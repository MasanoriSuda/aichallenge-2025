# Design

## Bound layers

For every receding-horizon sample, retain three independent layers:

1. Wall interval derived from the current planning/hard wall clearance.
2. Opponent-separation half-space while longitudinal body overlap is relevant.
3. Trust interval around the admitted Mission path.

The optimizer still uses the intersection of all three. Static-wall and lateral-acceleration post-validation is accepted when the corrected path remains inside layers 1 and 2, even if it exits layer 3. Exiting layers 1 or 2 remains a hard failure.

## Pass hysteresis

When current footprints are separated, a transient predicted overlap does not immediately restore opponent bounds. The existing `v2x_overtake_pass_predicted_overlap_confirm_sec` clock decides when the overlap becomes hard again.

## Diagnostics

Log soft trust releases separately from wall-bound and target-bound failures, including the first failing sample index.

