# Design

## Existing gap

The accepted receding-horizon path exports exact wall/opponent bounds, but
several fallback paths rebuild only `target_ey`.  The QP consequently falls
back to generic track bounds during the exact interval in which tactical
replanning is pending.  Return also uses the legacy horizon and therefore
loses the bound set immediately after rear-clear.

## Change

1. Make `evaluate_overtake_line_horizon()` the single owner of the base
   stage-wise wall corridor for execution horizons.
2. Store the same lower/upper bounds used to clip and physically validate each
   lateral target in `OvertakeLineHorizonEvaluation`.
3. Keep the current optimizer promotion as a refinement: a fully validated
   target-bound solution overwrites the base wall corridor with the tighter
   wall-plus-target interval.
4. Mark whether the exported corridor contains opponent bounds so diagnostics
   can distinguish `wall` from `wall+target` ownership.
5. Let target-bound physical holds and Return inherit the base wall corridor
   automatically instead of adding state-specific copies.

## Safety and racing policy

Walls remain hard throughout the Mission.  Opponent bounds remain hard when a
fresh feasible interval exists.  During an already admitted contact-tolerant
hold, an infeasible opponent interval is not fabricated or sent to OSQP; the
existing bounded hold/contact logic owns that interaction while the wall
corridor stays active.

This implements last-feasible continuity without treating stale opponent
geometry as current truth.

## Runtime validation

The existing stage-corridor line will additionally report
`target_bound=0/1`.  A healthy trial should show:

- no `active=0` while a Pass replan hold is active,
- `target_bound=0` during wall-only holds and Return,
- `target_bound=1` for fresh opponent-constrained horizons,
- no increase in OSQP failures or callback overruns.
