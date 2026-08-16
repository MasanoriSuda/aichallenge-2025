# Design

## Persistent solver boundary

Move OSQP ownership from the free per-call helper into a small reusable
`PersistentOsqpSolver` component. The component records matrix dimensions and
CSC row/column structure. When that signature matches, it updates `P`, `A`,
`q`, `l`, and `u` numerically. A changed signature discards the workspace and
performs a new setup.

An update API failure discards the possibly partially updated workspace and
performs one cold setup with the complete current problem. A failed solve also
discards the workspace so the next cycle cannot inherit a failed iterate.

## Warm start

Store only the last successful raw OSQP primal and dual vectors. For the
current MPC layout,

```text
variables   = state[0..N] + input[0..N-1]
constraints = dynamics/state[0..N]
            + box(state[0..N], input[0..N-1])
            + steering-rate[0..N-1]
```

shift every stage by one and repeat the terminal state/input/rate entry. Apply
the shifted vectors only when all dimensions and values are valid. This is a
solver warm start, not a new MPCC progress state.

## Diagnostics

Aggregate setup, update, warm-start, solve, iteration and failure counts in
the controller and report at most once per second. Report control callback
duration at the same bounded rate, with an explicit overrun count relative to
the configured 40 Hz period.

## Safety and compatibility

The existing post-solve finite and constraint-residual validation remains the
acceptance boundary. `OSQP_SOLVED_INACCURATE` retains its existing relaxed
residual check. Control output, topic types and command arbitration do not
change.
