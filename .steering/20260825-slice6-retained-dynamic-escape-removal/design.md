# Design

## Before

```text
fresh canonical DynamicEscape solution
  -> current-cycle command and prediction

deleted legacy producer
  X-> pending retained execution
      -> promote after wall admission
      -> cursor/identity lease
      -> restore at final publisher
```

The second graph is compiled and heavily tested, but its root producer is
absent. Runtime therefore always follows its `retained unavailable` branches.

## After

```text
fresh canonical DynamicEscape solution
  -> canonical permission
  -> current-cycle physical wall admission
  -> publish, hold current last steering, or request replan
```

No synthetic retained store replaces the deleted path. If a future retained
DynamicEscape is required, it must reuse the canonical certified-plan store
and same-fingerprint revalidation rather than resurrecting this private
`Eigen::VectorXd` cache.

## Deletion accounting

- Normal authority added: 0.
- Configuration added: 0.
- Dead producer/consumer chain removed: private retained execution, cursor,
  retained lease branch, promotion, restore and retained-only exit semantics.
- Live wall admission and DynamicEscape candidate generation retained.

## Failure behavior

When the fresh DynamicEscape candidate disappears, it no longer claims a
retained execution source. Existing current-cycle wall/exit logic may hold the
last finite steering or request a fresh replan; canonical solve failure still
uses the existing typed Emergency/fail-operational supervisor.
