# Design

The current variable transform is

```text
x_physical = D z
```

where each diagonal entry of `D` is the largest absolute finite box bound.
That normalizes a broad box around zero, but it does not normalize a narrow
trust region far from zero.  A refined progress interval such as
`[14.225, 14.275] m` becomes approximately `[0.9965, 1.0]`, while the same
width near the beginning of the horizon is much wider in solver coordinates.

The audit compares the exactly equivalent affine transform

```text
x_physical = o + D z
```

where `o` is the centre of each finite two-sided box and `D` is its half
width.  Fixed coordinates use their fixed value as the origin and neutral
scale.  Unbounded coordinates retain zero origin and neutral scale.

For the original QP

```text
min 0.5 x' P x + q' x
s.t. l <= A x <= u
```

the transformed data are

```text
Pz = D P D
qz = D (P o + q)
Az = A D
lz = l - A o
uz = u - A o
```

before applying the existing physical-row tolerance normalization.  This is
a coordinate change only; it does not alter the physical feasible set or
objective minimizer.
