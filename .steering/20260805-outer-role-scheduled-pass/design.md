# Design

## Observed failure

In `output/20260805-003612/d1/autoware.log`, seven missions entered Pass but no
mission completed cleanly. The last mission was committed as outside on
`side=-1`, later appeared as `inner_pass=1`, entered SafeSeparation on predicted
overlap, and ended in a persistent solver failure. No rolling outer transition
was accepted.

## Scheduled outer-role handoff

Initial full-mission preflight already reports the first path distance where a
committed outside side becomes inside. When continuous outer replan is enabled,
the current code deliberately does not reject that reversal, assuming the
rolling detector will repair it later.

The candidate will now retain:

- whether an outer handoff is required;
- the opposite side;
- Pass-relative handoff start and deadline distances.

The start is the later of body-clear and `deadline - max_shift_distance`. A
candidate is rejected if the body-clear point leaves less than the minimum
shift distance before the reversal.

During Pass, the committed schedule gets priority over the rolling detector.
It calls the existing atomic replacement preflight/commit path, which still
owns target separation, wall footprint, lateral acceleration, rear-clear and
generation checks. A failed attempt is retained and retried only at the
existing cooldown rate until its deadline.

## Robust final ranking

The complete preflight records:

- minimum center-path clearance to the lateral bounds;
- minimum available corridor width;
- minimum Return clearance.

The final cross-side mission selection compares the minimum of these physical
reserves before the progress score only when the difference exceeds the
existing side-quality advantage threshold. Small differences therefore keep
the aggressive rear-clear/speed ordering.

## Non-goals

- Arbitrary N-knot trajectory optimization.
- Pre-arming before a straight.
- Recovery/stuck state-machine changes.
- Parameter-only relaxation.

