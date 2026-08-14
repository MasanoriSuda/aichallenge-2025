# Design

## Pass-entry physical gate

At the ShiftOut completion boundary, resolve one of three actions:

1. `Inactive`: no soft wall warning; the existing fresh-horizon rule may enter Pass.
2. `HoldForReplan`: a soft wall warning exists but no physical/hard wall fault exists.
3. `Reselect`: the bounded hold has exceeded the existing Pass-horizon time or distance.

`HoldForReplan` disables the stale Frenet-DP execution prefix for that cycle and evaluates
a constant-current-lateral prefix against the normal planning wall reserve.  If necessary,
it may use only the already configured hard wall reserve.  A prefix that fails both checks
is not published.

The hold continues to request the existing fresh same-side candidate and center-contraction
paths.  When the warning clears, a fresh dynamic horizon may enter Pass normally.  When the
hold budget expires, the existing DynamicMissionWait path invalidates the old generation and
evaluates current/opposite alternatives.  Recovery remains reserved for physical contact,
hard wall-margin failure, unavailable wall samples, emergency front risk, solver recovery,
or a failed bounded reselect.

## Longitudinal authority

When the current body and predicted footprint sweep remain physically separated and the
current-side hold prefix passes wall validation, the controller retains at least the speed
present at gate entry.  This prevents a soft wall warning from becoming a Follow-speed
collapse.  No speed floor is applied when target prediction/body separation is unavailable.

## SafeSeparation lifecycle

`begin_pass_safe_separation()` becomes idempotent.  The first request initializes its clock,
distance budget and one-shot log.  Later requests while active leave those values intact.
This fixes both excessive logging and the effective infinite extension caused by resetting
the start time every control cycle.

## Dynamic verification

The next `make dev2` run should show:

- `Pass entry physical gate held` before either release/reselection;
- no `ShiftOut -> Pass` on the same cycle as a runtime wall preplan warning;
- zero physical wall contacts following the warning;
- one `SafeSeparation entered` line per continuous episode;
- no regression in `ShiftOut -> Pass` or clean Pass completion rates.
