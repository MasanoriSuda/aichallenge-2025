# Design

## Why a separate foundation

Changing the established five-state QP in place would simultaneously alter
state layout, input layout, row semantics, OSQP scaling, warm-start transport,
problem fingerprints, immutable plan extraction, retained revalidation, and
publication. That would make a regression impossible to localize.

This Slice adds one dependency-free mathematical module and its tests. The
existing controller does not link it. A later shadow Slice can assemble and
solve the six-state problem beside production without gaining authority.

## Continuous model

For state `x = [e_y, e_lag, e_psi, v, theta, delta]` and input
`u = [a, delta_dot, v_theta]`:

```text
e_y_dot   = v sin(e_psi)
e_lag_dot = v cos(e_psi) / (1 - k_ref e_y) - v_theta
e_psi_dot = v tan(delta) / wheelbase - k_ref v_theta
v_dot     = a
theta_dot = v_theta
delta_dot = delta_dot
```

The implementation uses first-order hold in the same equality convention as
the current extended MPCC:

```text
-x[k+1] + A_d x[k] + B_d u[k] = equality_offset
```

The steering-rate input has no instantaneous heading effect. It changes the
steering state; heading responds through that state. This is the representation
boundary missing from the current five-state formulation.

## Publication semantics

For a certified constant steering-rate stage:

```text
delta(t) = delta_0 + delta_dot * t
```

The pure sampler validates rate, time, and steering bounds, then returns both
`delta(t)` and `tan(delta(t))/wheelbase`. It does not clamp an infeasible
sample. A future canonical artifact will store sufficient state/input lineage
to call this semantic operation at the real publication cursor.

## Migration after this Slice

1. Shadow QP assembly with six-state row/scaling semantics.
2. Shadow solve and compare feasibility, runtime, wall proof, and 40 Hz
   publishability against the production five-state plan.
3. Define a new immutable formulation/schema and canonical artifact.
4. Obtain fresh and retained dynamic acceptance without publication.
5. Promote authority atomically and delete the five-state production path in
   the same migration Slice.

No mixed permanent authority is permitted.
