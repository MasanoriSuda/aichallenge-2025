# Design

## Existing behavior

The controller already has separate inner/outer hard-curve exceptions, but the policy is spread
across completion, forbidden-zone, side-vehicle, and continuation gates. Side preference is also
implicit: enabling inner entry makes any minimally feasible inner gap the first choice. This makes
the intended race policy difficult to verify and lets conservative legacy gates appear as a
generic `overtake hard curve blocked` result.

## Curve attack arbitration

Introduce a pure, tested curve-side selector. For an unlocked maneuver it receives the curve's
inside sign, execution-feasible left/right corridors, and their maximum continuous usable path
length. It selects:

1. the inside when its continuous corridor meets the configured minimum;
2. otherwise the outside when it is executable;
3. otherwise no side.

An already locked side remains authoritative and is never changed by a later curve-side
preference. The existing inner/outer entry and hard-continuation resolvers continue to own the
execution gates, so hard curvature itself is not a categorical rejection while explicit safety
and geometry gates remain in force.

The behavior FSM may enter `Overtake` one V2X callback before the control timer creates the
explicit ShiftOut line. Entry versus continuation is therefore keyed to an active locked
ShiftOut/Pass line, not to the behavior label alone. Until that line exists, a repeated curve
assessment remains an entry; after it exists, only same-side locked continuation is legal. This
prevents the entry/continuation handoff from producing a one-cycle `Overtake -> Follow` and
arming the legacy curve cooldown.

The pre-lateral-clear target-ordering guard is limited to a configurable near-target distance.
At longer range, ego is still behind and may move laterally across the target's current ordering
to enter the validated vehicle-to-wall corridor. The inflated corridor and lateral-reachability
checks remain authoritative there. The dev3 profile activates ordering cancellation within
2.0 m; missing configuration retains the former unbounded guard.

## Corridor measurement

For each gap-planner result, sum reference-path segment lengths across consecutive active samples
whose post-inflation corridor width meets the existing overtake minimum. Store the longest run in
the side assessment. The dev3 profile uses 3.0 m, within the requested few-metre confirmation
window. A fallback lateral target without a validated continuous interval does not qualify as an
inside dive.

## Scope

Changes stay in `multi_purpose_mpc_ros`: the V2X policy helper, controller integration, YAML
profile, tests, and MPC integration specification. No evaluator or interface contract changes.
