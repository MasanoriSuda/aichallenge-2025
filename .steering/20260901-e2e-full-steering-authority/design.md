# Design

The network remains a frozen-base spatial adapter, but the residual is treated
as an implementation of total steering ownership rather than a permanently
small trim.  The composed command is:

```
steering = clip(base_steering + learned_correction, -1.0, 1.0)
```

With a `1.2 rad` correction range, the learned policy can cancel and reverse a
wrong frozen-base command.  Base steering remains an explicit model feature,
so the correction target is well-defined for identical LiDAR scans.

This Slice does not add an emergency trigger or conditional takeover rule.
The existing explicit authority switch remains the only rollback boundary:
shadow-only keeps base output bit-for-bit; authority mode uses the same learned
function on every fresh admitted scan.  Dynamic acceptance, not a local case
guard, decides promotion.

Near-bound occupancy is reported at 95% of the configured authority limit.
Exact clipping and near-bound residency are different signals and must not be
conflated.
