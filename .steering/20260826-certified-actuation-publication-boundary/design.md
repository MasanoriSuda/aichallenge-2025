# Design

## Observed causal chain

```text
six-state QP solves a stop/hold boundary with a small lower-bound residual
  -> immutable artifact accepts it under physical_global_tolerance
  -> exact physical trajectory accepts predicted velocity under its sealed
     velocity lower-bound tolerance
  -> current-world wall/dynamic/actuation proof accepts the exact plan
  -> production adapter reconstructs CanonicalActuation from raw values
  -> build_canonical_normal_command requires exact nonnegative speed/vtheta
  -> command-rejected
  -> canonical Emergency becomes the next predecessor
```

The first abnormal post-start decision is therefore not a wall, V2X, solver,
Recovery, or tactical failure.  It is a producer/consumer mismatch at the last
publication boundary.

## Selected repair

The production adapter owns the only conversion from certified numerical
states to the physical command representation:

- predicted velocity uses the exact physical trajectory's sealed velocity
  lower-bound tolerance;
- virtual progress speed uses the execution artifact's sealed physical global
  tolerance;
- a finite value in `[-tolerance, 0)` becomes exact physical zero;
- a value below `-tolerance` is rejected;
- positive values and all other actuation fields are unchanged.

The same velocity projection is applied to the compatibility speed horizon so
the canonical command and `current_control` cannot disagree.  The raw artifact,
dynamics, wall proof, dynamic proof and solution identity are not rewritten.

## Alternatives rejected

- Allow negative command speed: leaks solver numerical representation into the
  vehicle interface and later horizon validation still rejects it.
- Unconditional clamp: hides an uncertified violation.
- Reintroduce a solver inset for stop singletons: recreates an empty interval.
- Emergency/legacy fallback: masks the canonical contract defect and adds a
  second normal authority.
