# Design

## Scope

This change is deliberately narrower than a new overtake strategy. It repairs
the handoff between live Pass optimization and the already validated physical
same-side prefix.

## State machine

1. A target-bound-only optimizer failure occurs during frozen `Pass`.
2. Revalidate the current/previous same-side prefix against the live wall and
   lateral-acceleration constraints.
3. If all hard guards pass, retain Pass speed ownership and execute the prefix
   while the optimizer retries each cycle.
4. Track one cumulative time/distance budget across intermittent feasible
   optimizer results.
5. Start a clear-stability timer when fresh feasible horizons resume.
6. Release the cumulative prefix state only after feasible horizons remain
   continuous for the configured stability time.
7. A renewed target-bound failure resets only the clear-stability timer. It does
   not reset the original time/distance budget.
8. Any hard fault or budget exhaustion ends the hold and leaves the existing
   fallback/phase logic in control.

## Parameters

- `v2x_overtake_receding_horizon_target_bound_prefix_enabled`
- `v2x_overtake_receding_horizon_target_bound_prefix_max_sec`
- `v2x_overtake_receding_horizon_target_bound_prefix_max_distance`
- `v2x_overtake_receding_horizon_target_bound_prefix_clear_stable_sec`

Initial values are 1.5 s, 8.0 m, and 0.20 s. The 1.5 s / 8.0 m budget is
aggressive enough to cover the observed 0.31 s / 1.64 m failure while remaining
strictly bounded. Hard wall and emergency checks are unchanged.

## Expected log change

- A target-bound hold should log a `1.50 s/8.00 m` limit.
- One-cycle feasible results should no longer emit repeated resolved/started
  pairs.
- A resolution log should include the stable-clear duration.

