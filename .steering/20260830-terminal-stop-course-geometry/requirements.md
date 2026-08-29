# Requirements

## Objective

Eliminate false `terminal-contingency-unavailable` authority loss caused by
building a maximum-braking Stop successor from the short executable prefix's
geometry after that prefix has ended.

The correction must preserve the existing exact current-world wall, dynamic
obstacle and Follow proof chain.  It must not relax a clearance, solver
tolerance, braking limit, continuity gate or publication rule.

## Frozen evidence

Baseline: `73f94910`.

Dynamic run: `output/20260830-011957/d1/autoware.log`.

Representative failures:

- decision 1726: speed 5.83 m/s, publisher interval clear, terminal Stop
  rejected at exact sample 87 with `invalid-lateral-bounds`;
- decision 1887: speed 5.54 m/s, terminal Stop rejected at sample 27;
- decision 5664: speed 5.43 m/s, terminal Stop rejected at sample 67.

In every example the exact Stop model rejects before the authoritative wall
grid or dynamic-obstacle proof runs (`wall_valid=0`, checked count zero).

## Root-cause contract

An `ExecutionArtifact` contains only `execution_prefix_steps` controls and
lateral bounds.  `build_stop_contingency()` currently samples curvature and
bounds from that prefix for the complete braking distance.  Once progress
passes the prefix, the sampler clamps to the last prefix stage and silently
extrapolates its curvature and lateral interval.

The resulting Stop suffix is neither the full planning course geometry nor an
exact statement of the physical wall corridor.  A path-tracking Stop can
therefore be rejected against an expired tactical interval without reaching
the current-world footprint proof.

## Required behavior

1. The executable normal prefix remains short and immutable.
2. The certified physical source separately owns full-horizon Stop course
   progress, curvature and physical lateral support.
3. Terminal Stop synthesis samples that distinct geometry and fails closed
   when braking progress is outside its sealed horizon; it never extrapolates
   the final prefix stage.
4. The same maximum-braking/path-tracking policy remains shared by proof and
   publisher.
5. Exact wall-grid, dynamic-obstacle and Follow proofs remain mandatory before
   one publisher interval receives normal authority.

## Prohibited changes

- no Mission resume/lease/grace/timeout/fallback additions;
- no wall, vehicle, solver or continuity threshold changes;
- no wider synthetic bound inserted merely to make validation pass;
- no production authority change;
- no modification or staging of generated `output/`, result JSON, snapshot or
  symlink-manifest artifacts.

## Falsifiers

- If a failure occurs before progress reaches the executable-prefix end, the
  prefix extrapolation hypothesis is insufficient.
- If full physical course support rejects the same Stop, that case is a real
  physical infeasibility rather than this defect.
- If exact Stop succeeds but wall/dynamic proof rejects, retain fail-closed
  behavior and classify the physical blocker.

