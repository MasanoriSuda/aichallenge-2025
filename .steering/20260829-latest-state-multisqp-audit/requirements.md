# Requirements

## Objective

Evaluate architecture arm D after C proved that a reachable candidate can
solve while the first exact nonlinear proof still rejects.

## Contract

- start from C's reachable nonlinear candidate;
- keep one immutable semantic suffix and every hard row unchanged;
- after each solved QP, rebuild only temporal Frenet tangents around that
  result and solve the same problem again;
- run only for an explicit bounded iteration count in offline/observation API;
- stop immediately when exact artifact and physical-adapter proof accept;
- report numerical solve count and final proof result.

## Prohibited changes

- no production authority or Store connection;
- no live retry/fallback semantics;
- no tolerance, iteration-limit, cost, bound or clearance changes;
- no alternate problem after a failure;
- no clamping of solved states or commands.

## Acceptance

- iteration limit one is identical to C;
- the deterministic C proof failure is either accepted by later SQP
  corrections or remains an explicit D failure;
- every iteration preserves the same costs and physical constraint rows;
- all tests and package regressions pass.
