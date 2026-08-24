# Design

## Physical interval

For semantic steering `delta0`, physical limit `D`, rate limit `R` and first
stage duration `dt`, the exact constant-rate interval is:

```text
physical_lower = max(-R, (-D - delta0) / dt)
physical_upper = min( R, ( D - delta0) / dt)
```

This is an algebraic consequence of
`delta(dt) = delta0 + rate * dt`; it is not a tuned guard.

## Solver-certificate interior

For OSQP physical row tolerance `abs + rel * |bound|`, choose a symmetric
margin using the maximum physical rate magnitude:

```text
margin = (abs + rel * R) / (1 - rel)
solver_lower = physical_lower + margin
solver_upper = physical_upper - margin
```

Because any shifted bound has magnitude at most `R + margin`, its accepted row
residual is at most `margin`. A certified solution therefore cannot cross the
original physical interval. If the interior is empty, the adapter rejects the
problem rather than clamping or silently relaxing the limit.

## Ownership

- semantic current steering owns the reachability origin;
- the adapter owns the exact first-stage interval transformation;
- the persistent solver owns the absolute/relative certificate values;
- the certified sampler remains the final publication-time physical check.

Later stages retain the ordinary rate box because only the first stage can be
published before the next receding-horizon solve.
