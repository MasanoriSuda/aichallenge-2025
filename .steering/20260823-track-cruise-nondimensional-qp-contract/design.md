# Design

## Coordinate contract

Let the physical decision vector be `z`, the dimensionless vector be `y`, and
positive diagonal variable and row scales be `S` and `R`:

```text
z = S y
```

The physical QP

```text
min 0.5 z' P z + q' z
s.t. l <= A z <= u
```

is passed to OSQP as

```text
P_s = S P S
q_s = S q
A_s = R A S
l_s = R l
u_s = R u
```

The primal and dual mappings are:

```text
y          = S^-1 z
lambda_s   = R^-1 lambda
z          = S y
lambda     = R lambda_s
```

This is an invertible coordinate change.  It changes neither the physical
objective nor the feasible set.

## Semantic scales

One characteristic scale is selected for each five-state dimension and each
input dimension, then repeated for all stages:

- lateral, lag, heading, velocity, local progress;
- acceleration, curvature, virtual-progress speed.

Finite declared physical bounds determine the characteristic magnitude.
Unbounded or zero-width dimensions use their explicit base physical unit.
The same dimension never changes scale between stages.

Constraint rows use the reciprocal scale of the physical quantity represented
by that row:

- each dynamics equality uses its state dimension;
- each variable box uses its variable dimension;
- each curvature-rate row uses curvature.

## Adapter ownership

`PersistentOsqpSolver` owns the algebraic transformation because it owns the
solver workspace and warm-start coordinate.  It accepts an optional immutable
scaling contract per solve, validates it before setup/update, maps warm starts
into solver coordinates, and maps the result back to physical coordinates.

The dedicated Track/Cruise caller supplies the five-state semantic scaling.
All other callers omit it and remain byte-for-byte on the current path.

The adapter recomputes residual/tolerance telemetry from the original physical
problem after mapping.  Downstream semantic and wall checks therefore see no
new unit system.

## Why this differs from the rejected row experiment

The earlier candidate multiplied constraint rows but left decision variables,
objective and parts of warm-start provenance in the original coordinates.
That reduced row rejects but worsened conditioning, wall proof and real-time
behavior.  This Slice treats the QP and both warm-start vectors as one
coordinate system.  If that still fails dynamically, the whole candidate is
removed; it will not be rescued by solver tuning.
