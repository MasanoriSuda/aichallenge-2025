# Design

The spatial model remains an observer unless the explicit authority flag is
true.  On each sample with a fresh speed and valid finite inference:

```text
published steering = clip(candidate3 steering
                          + clip(spatial residual, -0.12, +0.12))
```

If inference is skipped or rejected, the applied correction is exactly zero and
candidate3 is published.  A residual checkpoint and spatial authority are
mutually exclusive so only one learned correction owner exists.

Runtime metrics distinguish inferred residual from applied correction and count
authority application/clipping.  The existing shadow gate remains valid for
performance and freshness; an authority run additionally proves explicit launch
enablement and the 0.12 rad bound.
