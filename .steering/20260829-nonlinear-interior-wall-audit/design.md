# Design

For transition `i` and physical substep time `tau`, the canonical model gives

```text
x(tau) = f_tau(x_i, u_i)
```

The audit numerically linearizes the same function used by the exact proof:

```text
f_tau(x, u) ~= A_tau x + B_tau u - offset_tau
```

and appends only its lateral row:

```text
lower(tau) + offset_tau
  <= A_tau[lateral] x_i + B_tau[lateral] u_i
  <= upper(tau) + offset_tau
```

`lower(tau)` and `upper(tau)` are the same interpolation of the frozen stage
endpoint intervals used by the physical adapter. The existing affine swept
rows remain, making E strictly additive.

The existing C/D observation solver is extended by one explicit audit mode.
Each SQP correction rebuilds the standard temporal dynamics and the added
interior rows around the same new primal. The immutable semantic
`AssemblyRequest` remains the artifact source, and the existing exact
physical adapter remains the sole acceptance authority.
