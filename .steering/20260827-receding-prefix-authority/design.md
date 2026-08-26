# Design

## Contract

The immutable certified artifact remains the only normal authority source.
Retained current-world validation produces one of two proof scopes:

- `FullSuffix`: delay path and the complete remaining suffix are clear.
- `CurrentStagePrefix`: delay path and the first remaining execution stage are
  clear, while a later nonlinear continuation or static-wall suffix failure
  requires the next receding solve.

`CurrentStagePrefix` is not a grace period or a second controller. It publishes
the same first actuation from the same certified artifact. It may not bypass a
blocked delay path, blocked current stage, dynamic obstacle, Follow hard gap,
unreachable actuator state, stale identity or invalid artifact. The certified
trajectory, stage-end velocity/steering traces and executable stage count are
all truncated to that same first-stage proof; metadata may not advertise a
longer authority horizon than the physical evidence contains.

Fresh current-decision artifacts still require the complete remaining horizon.
Only an immutable retained artifact that has been replayed from the current
physical state may receive finite-prefix authority. The next control cycle
must replace that transaction with a fresh or newly revalidated artifact; age
alone never extends it.

The current stage remains fail-closed for nonlinear trajectory, actuator,
wall, dynamic-obstacle and Follow-gap failures. Later dynamic-obstacle or
Follow-gap failures are not relaxed by this Slice because no dynamic evidence
established an equivalent safe-prefix contract for them.
