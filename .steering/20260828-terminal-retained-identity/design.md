# Design: terminal retained execution identity

## Root cause

The caller considered an executed artifact eligible whenever its cursor was
available and its identity matched the last published intent.  It calculated
whether the live Mission matched, but used that result only to obtain traveled
distance.  The resolver therefore selected the retained identity even after
the live OvertakeLine FSM had completed Return and entered Idle.

Every replacement problem inherited Return from the retained artifact.  Once
published, that replacement became the next retained artifact, so cursor
expiry could never terminate the chain while solves kept succeeding.

## Repair

Keep artifact executability and live tactical identity as separate inputs:

- `retained_execution_active`: exact published artifact still has a cursor;
- `retained_execution_matches_live_tactical_state`: target, generation,
  homotopy and phase still match the live non-terminal Mission.

The pure resolver may select `RetainedExecutedArtifact` only when both are
true.  An executable but superseded artifact reports
`RetainedExecutedArtifactSuperseded` and contributes no new tactical
authority.  This deletes the recursive authority edge rather than adding a
timer or fallback.

## Architecture comparison

- A, persistent Mission lifecycle: failed because artifact identity survived
  Return -> Idle and recursively generated Return successors.
- B, stateless current-world bundle: does not contain Return after the live
  terminal transition, so the recursive edge is absent.
- C/D are not required for this failure family; the exact seven-state solver
  repeatedly solved the synthetic Return problems, so candidate generation or
  nonlinear feasibility is not the first violated invariant.

