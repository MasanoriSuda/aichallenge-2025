# Design

## Observed problem

`v2x_overtake_gap_lookahead_distance` was converted to points with
`reference_path.resolution` (0.6 m). The runtime trajectory is spaced at about
1 m, so 30 m became 52 points and roughly 51-52 m of actual course. A distant
curve could therefore select the current tactic and make a locally open pass
less decisive.

SafeSeparation separately confirmed `TargetClearAhead` at 2 m, while the only
speed-preserving Return helper required `max(front_clear, reselect_min) = 4 m`
and retained pass-side future-sweep requirements. This caused avoidable wait or
FollowPrepare transitions after the target was already confirmed ahead.

## Changes

1. Replace resolution-based point calculation with cumulative waypoint arc
   length. Reuse it for the normal 30 m gap plan and full static Mission
   validation.
2. Add a 20 m tactical horizon to Frenet DP. All 30 m samples still constrain
   the continuous path and soft reserve, while curve classification, tactical
   reference and inside-radius cost stop at 20 m.
3. Add a dedicated confirmed-target-clear Return predicate. It requires target
   continuity, current body separation, an available Return corridor and no
   hard fault, but does not require the obsolete pass-side future sweep to stay
   clear.
4. If a proposed Return fails full runtime validation, explicitly continue to
   DynamicMissionWait/FollowPrepare instead of silently remaining in Pass.

## Expected effect

- The path search remains continuous and feasibility-aware without allowing a
  far hairpin to dominate the immediate overtake tactic.
- A completed forward separation connects to Return sooner and preserves speed
  instead of waiting for an unrelated 4 m reselect threshold.

## Remaining dynamic verification

- Confirm `points` and `distance` in the rolling DP log are about 30 m.
- Compare `Pass -> Return -> Idle` count against `Pass -> FollowPrepare` and
  `Pass -> Recovery`.
- Check that wall/contact events do not increase.
