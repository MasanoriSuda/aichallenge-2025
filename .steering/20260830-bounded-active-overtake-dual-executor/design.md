# Design

## Cause addressed

The evidence Slice proved that a sibling can be certified before tactical
abandonment, but its implementation created one child thread for each active
dual evaluation. The measured background tail reached 471.697 ms. Keeping
that implementation while promoting sibling authority would mix an authority
fix with an unbounded scheduling regression.

## Structure

Introduce a single persistent one-job executor:

```text
LatestOnlyWorker (normal solve epoch)
  |-- submit negative branch to persistent executor
  |-- solve positive branch locally
  `-- join exactly the submitted negative ticket
```

The executor has no backlog and exposes submission/completion identity. A job
exception is returned explicitly. Its lifetime is owned by `MPC`, and it is
stopped before captured solver contexts are destroyed.

## Non-goals

- selecting the opposite side;
- modifying Mission/no-return state;
- reducing horizon or proof scope;
- changing worker cadence.
