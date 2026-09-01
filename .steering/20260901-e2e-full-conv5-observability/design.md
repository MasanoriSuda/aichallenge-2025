# Design

The current production adapter flattens the frozen base network's conv5 map
(`64 x 17 = 1,088`) and applies a fixed Gaussian projection to 128 dimensions.
The previous Slice showed that this projection places 60% of a known
four-vehicle failure tail inside the normal p50 envelope even though a physical
geometry representation places 0% there.

Add a diagnostic `static_conv5_full_base` representation:

```text
full frozen conv5 map (1,088)
+ synchronized wheel speed
+ embedded frozen-base steering
```

For classifier comparison, standardize each dimension using train data exactly
like every existing probe.  For nearest-neighbour observability, standardize
the full conv5 map using successful normal data before deriving the same
cross-run p50/p95 scale.  This is a diagnostic metric, not a runtime transform.

The experiment is accepted only if full conv5 improves three-seed balanced
accuracy without increasing production-normal false-material actions and keeps
peer/focus-tail direction.  Otherwise the projection is not actionable despite
the nearest-neighbour evidence.
