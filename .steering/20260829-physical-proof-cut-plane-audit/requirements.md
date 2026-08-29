# Requirements

## Objective

Determine whether the frozen ShiftOut wall-proof failure can be closed by
adding only the exact physical proof's violated nonlinear interior sample,
rather than adding every 10 ms sample to one large QP.

## Frozen evidence

- the unchanged C/D problem solves but ShiftOut sequence 1266 fails exact
  proof at dense stage 339 by about 2.28 mm;
- dense audit E adds 336 rows and reaches the unchanged 4000-iteration limit;
- Follow sequence 531 proves that nonlinear interior rows can close the exact
  proof without changing tolerances;
- Cruise sequence 601 fails before physical proof and is a control case, not a
  wall cut source.

## Contract

- start from the same reachable, time-aligned C candidate;
- solve and run the unchanged exact physical proof;
- only for `invalid-lateral-bounds`, map the rejected dense sample back to its
  exact transition stage and substep;
- append one tangent constraint for that exact sample and retain all previous
  cuts;
- relinearize canonical endpoint dynamics and retained cuts around the new
  primal;
- stop on acceptance, a non-wall proof failure, duplicate/endpoint cut,
  solver rejection or a fixed audit iteration bound;
- expose no Store, worker, publisher, command or production authority path.

## Prohibited changes

- no solver iteration, tolerance, clearance, cost or wall-margin change;
- no use of a rejected iterate as an executable artifact;
- no physical-proof bypass;
- no Mission rule, lease, timeout, grace period or fallback;
- no claim of production readiness from an offline success.

## Acceptance

- exact dense-stage-to-transition mapping is unit tested at boundaries;
- selected-cut assembly preserves every original row and bound as an exact
  prefix;
- every added cut identifies one proof rejection sample;
- immutable ShiftOut and Follow snapshots are classified against D/E;
- full build and package regression pass.
