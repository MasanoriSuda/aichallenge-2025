# Design

## Root cause

`update_overtake_line()` computed the runtime completion rollout near the start
of the active Pass path.  That rollout uses the same speed-cap profile and the
full-speed Pass coupling that owns longitudinal execution.

Later in the same callback, rear-clear horizon revalidation rebuilt almost the
same mission and ran a second rollout.  The second path used the nominal
Mission closing speed instead of the execution-coupled closing speed and used
different completion reserves.  The two predictions therefore disagreed even
though they described the same current state.

## Changes

1. Record the remaining lateral-transition distance used by the runtime
   completion rollout.
2. Add a small pure resolver that converts the shared runtime completion result
   into an absolute Pass-origin rear-clear requirement.
3. Delete the duplicate horizon rollout and feed the pure resolver result into
   `resolve_rear_clear_replan_window()`.
4. Add tests for the absolute-distance conversion and fail-closed behavior.

## Safety

- The pure resolver does not make a new feasibility decision.  It propagates
  the existing runtime rollout and dynamic-distance decision.
- The static committed horizon, revalidation lead, absolute Pass limits and
  hard physical guards remain unchanged.
- A stale dynamic horizon or unavailable/infeasible runtime prediction remains
  unchecked or infeasible; it is never promoted to a valid extension.
