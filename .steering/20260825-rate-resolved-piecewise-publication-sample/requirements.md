# Requirements

## Objective

Produce the 40 Hz steering command at its immutable publication time even when
that time crosses one or more variable-duration MPCC prediction stages.

## Root cause

Run `output/20260825-012740` consumed 5,045 physically certified
rate-resolved QPs. The first-rate reachability repair removed every steering
boundary rejection, but 11 results were still rejected solely because the
25 ms publication time was later than the first prediction-stage duration.

The QP already certifies a piecewise constant steering-rate sequence over the
whole horizon. The current sampler discards that information and assumes the
publisher must remain inside stage zero.

## Scope

- Replace the certified single-stage publication API with a certified
  piecewise-rate sequence API.
- Integrate immutable stage durations and solved steering rates until the
  publication boundary.
- Check the actual semantic steering at every crossed boundary and at the
  publication point.
- Reject only when the publication time exceeds the whole certified horizon
  or an actual integrated steering is invalid.
- Record sampled stage, in-stage elapsed time and horizon duration.

## Non-scope

- No prediction-stage duration, publish frequency or horizon change.
- No interpolation of unrelated curvature endpoints.
- No clamp, tolerance relaxation, fallback or authority promotion.
- No parameter tuning.

## Preserved user state

`aichallenge/result-summary.json` is user-owned and must not be edited, staged
or committed.

## Rollback

Rollback target: `8dd2ad3`.
