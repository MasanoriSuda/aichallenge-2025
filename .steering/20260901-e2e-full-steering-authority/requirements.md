# Requirements

## Objective

Determine whether the admitted LiDAR teacher can be represented and executed
when it must fully override the frozen base steering near a dynamic obstacle
or wall.

## Root cause from the preceding Slice

The conditioned v10 candidate chose the correct escape direction but was
limited to `+/-0.12 rad`, while the teacher required `0.83--0.88 rad` in the
failed NPC episode.  No threshold or timeout change can make that command
space feasible.

## Requirements

- retain only 2D LiDAR and wheel speed as external ML inputs;
- preserve the immutable embedded base model and its startup provenance check;
- train a base-conditioned candidate over the full `+/-1.2 rad` correction
  range, which is sufficient to express total steering ownership after final
  steering clipping;
- keep production defaults and the packaged production checkpoint unchanged;
- require strict aggregate, held-out, peer and normal-anchor offline gates;
- evaluate in shadow before granting any steering authority;
- then require single-vehicle and NPC seed 2026/2027 closed-loop acceptance;
- reject the candidate rather than add scenario-specific gates, leases,
  timeouts or clearance patches.

## Definition of Done

- diagnostics expose continuous near-bound residency;
- offline gates pass without unacceptable normal-driving leakage;
- shadow and authority runs have no stale/error/coverage defect;
- three laps complete without penalty or stall in single and both NPC seeds;
- only a fully qualified artifact may replace production defaults.
