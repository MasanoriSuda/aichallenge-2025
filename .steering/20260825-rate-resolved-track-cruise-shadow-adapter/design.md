# Design

## Why this Slice exists

The rate-resolved QP assembler deliberately knows nothing about the legacy
five-state formulation. Connecting it directly inside the controller would
duplicate reference, bound and weight interpretation in a second runtime path.
This adapter freezes that interpretation first as a pure, testable contract.

## Mapping

For state stage `k`, the first five fields remain
`[e_y,e_lag,e_psi,v,theta]`. Stage zero steering is the measured actuator
state. The same observation snapshot, not a separately supplied nominal
reference, also owns the first five stage-zero fields and the first dynamics
linearization anchor. For `k>0`, steering reference and bounds come from legacy curvature
input stage `k-1`:

```text
delta_ref = atan(L * kappa_ref)
delta_box = atan(L * kappa_box) intersect global steering box
```

Around `delta_ref`,

```text
d(kappa)/d(delta) = sec(delta_ref)^2 / L
```

is used to preserve the local quadratic curvature cost in steering units.
The old adjacent-curvature cost represents a per-stage curvature change. In
the new actuator model that physical quantity is `delta_dot * dt`, so it maps
to a steering-rate magnitude weight. It is not retained as another rate row or
as a delta-of-steering-rate cost.

Acceleration and virtual-progress input references, boxes, direct costs and
input-delta costs map unchanged. The steering-rate reference is zero; steering
state tracking carries the desired curvature and the QP may choose any bounded
rate needed to reach it.

## Later runtime boundary

A later Slice may populate this snapshot from the existing Track/Cruise
problem builder, submit it to a latest-only worker and compare the resulting
solution in shadow. That Slice must not let the new result participate in
canonical selection until its timing and physical certificates pass.
