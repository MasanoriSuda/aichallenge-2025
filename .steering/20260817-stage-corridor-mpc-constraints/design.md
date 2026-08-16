# Design

## Current split

The live overtake optimizer constructs stage-wise wall and opponent bounds,
optimizes a lateral target sequence inside them and physically revalidates the
result.  `OvertakeLineOutput`, however, exports only `target_ey`.  The tracking
MPC then uses that sequence as `xr` while its hard `e_y` bounds remain the
generic track bounds.

This permits the tracking solution to leave the corridor that admitted the
overtake path, after which actual-footprint and target/wall guards revoke the
Mission.

## Change

1. Add a small, testable core resolver that intersects base MPC bounds with an
   optional stage corridor and clips the soft reference into the intersection.
2. Extend the overtake horizon/output records with stage lower/upper arrays
   and an explicit activation bit.
3. After a receding-horizon candidate passes physical revalidation, rebuild
   its hard per-stage bounds from the exact wall reserve and target constraints
   used by that accepted validation attempt.
4. In `init_problem()`, immediately after `update_overtake_line()`, intersect
   the main MPC `lb/ub` with those stage bounds before filling `xmin_dyn`,
   `xmax_dyn` and `xr`.
5. Keep the DP/optimized `target_ey` as the soft reference.  No new phase,
   Mission latch or recovery rule is introduced.

## Failure behaviour

- No active/fresh stage corridor: preserve current generic MPC bounds.
- Contradictory active corridor: throw from MPC preflight and use the existing
  bounded solver-failure fallback; never silently drop a hard opponent bound.
- OSQP failure: existing persistent-solver reset/fallback remains authoritative.
- Target loss: retain current continuity and last-observation rules.  No
  target-loss-to-Return shortcut is added.

## Runtime evidence

Add a throttled debug line containing whether stage constraints are active,
the applied sample count and minimum corridor width.  Trial validation should
compare:

- OSQP failures and callback overruns,
- `optimized horizon failed physical revalidation`,
- target/wall Recovery counts,
- `Pass -> Return -> Idle` completion count.
