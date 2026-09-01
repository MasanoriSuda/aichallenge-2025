# Requirements

## Objective

Remove the qualified spatial authority's clean-track pace regression without
weakening the dynamic-obstacle correction range or adding a runtime trigger.

## Observed regression

Compared with the same v11 model in shadow, single-vehicle authority:

- took about `5.87 s` longer over the measured run;
- travelled about `9.95 m` farther;
- reduced mean forward speed by about `0.046 m/s`;
- increased steering-command total variation by about `11.6%`;
- emitted a material correction on about `8.3%` of clean-track LiDAR frames,
  predominantly opposing the frozen base steering.

Both runs commanded constant `+0.6 m/s2` acceleration and had no safety-brake,
penalty or stall.  The first hypothesis is therefore normal-state spatial
leakage, not longitudinal arbitration.

## Constraints

- keep production artifact and launch defaults frozen during this Slice;
- keep the `+/-1.2 rad` model and authority representation;
- add no runtime threshold, obstacle trigger, lease, timeout or special case;
- use only admitted, penalty-free, base-authority/shadow single-vehicle data as
  new zero-residual normal anchors;
- preserve immutable run identity and train/validation separation;
- require the existing aggregate, peer, held-out, normal and frozen-failure
  gates before any closed-loop candidate test.

## Definition of Done

- a current-distribution normal-anchor corpus is reproducible;
- a candidate reduces clean-track material correction and normal leakage;
- dynamic teacher direction and the frozen v10 failure correction remain
  representable;
- only a candidate passing offline gates proceeds to shadow and authority A/B;
- production remains unchanged if the trade-off is not strictly better.
