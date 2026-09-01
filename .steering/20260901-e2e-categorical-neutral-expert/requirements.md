# Requirements

## Objective

Determine whether an explicitly learned neutral steering expert can remove
normal-state intervention while retaining the full-range correction required
by the frozen wall-stall failure.

## Root-cause evidence

The v12/v13 audit showed a monotonic trade-off: adding normal anchors reduced
clean correction but also removed the large correction needed in the failure
suffix.  More sampler or loss-weight tuning is prohibited in this Slice.

## Constraints

- keep v11 and every production runtime default frozen;
- keep the same dataset, physical inputs, frozen base, representation and
  `+/-1.2 rad` correction range;
- change only the learned output contract from soft probability composition to
  categorical neutral/left/right selection with conditional side magnitude;
- do not add a confidence threshold, obstacle trigger, debounce or deadband;
- evaluate the exact winner-take-all decode offline;
- require aggregate, peer, independent-normal, held-out focus and frozen
  failure-tail evidence before any closed-loop run;
- reject the candidate without runtime integration when offline evidence fails.

## Definition of Done

- one reproducible categorical-expert candidate is trained;
- exact winner-take-all decoding is evaluated against all frozen evidence;
- clean intervention and failure-tail correction are compared with v11;
- production is unchanged unless the candidate is strictly admissible;
- the result is documented and committed without generated artifacts.
