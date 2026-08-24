# Design

## Boundary

The assembler accepts already-linearized stages plus explicit stage-major
references, bounds, and weights. It deliberately does not know about ROS,
Mission, V2X, wall cache, problem fingerprints, asynchronous workers, or
canonical publication. Those adapters belong to later shadow Slices.

## Variable layout

```text
z = [x_0 ... x_N, u_0 ... u_(N-1)]
x = [e_y, e_lag, e_psi, v, theta, delta]
u = [a, delta_dot, v_theta]
```

## Constraint layout

```text
rows [0, 6(N+1)):
  state zero and affine dynamics equalities

rows [6(N+1), 6(N+1) + variable_count):
  one identity box row per physical variable
```

No steering-rate difference row is needed to make commands physically
reachable: `delta_dot` itself is the actuator and its input box is the hard
rate constraint. Optional steering-acceleration comfort belongs to the cost,
not a second physical owner.

## Cost

Each state and input receives an explicit non-negative diagonal reference
weight. Input-delta weights add

```text
0.5 w (u_0 - u_previous)^2
0.5 w (u_k - u_(k-1))^2
```

per input coordinate. This preserves a convex upper-triangular OSQP Hessian
and does not introduce a hidden clamp.

## Next boundary

After this contract passes, a controller adapter may construct the same shadow
problem from existing progress geometry and compare it with the production
five-state solve. It may log results but cannot participate in authority until
fresh/retained physical proof and runtime acceptance are complete.
