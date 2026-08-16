# Design

## Boundary ownership

`OvertakeLineHorizonEvaluation` will carry two lateral envelopes:

1. wall-only bounds produced by the common physical horizon evaluator,
2. effective bounds, which may be tightened by a fresh opponent prediction.

The tracking MPC selects between them through a small stateful gate. This
removes the previous ambiguity where the effective vector was overwritten and
the wall-only fallback was no longer available at solve time.

## Asymmetric target-bound gate

Opponent-bound promotion is deliberately asymmetric:

- enable only after `confirm_sec` of uninterrupted availability,
- disable immediately on candidate loss,
- disable immediately after a solver fallback,
- remain disabled for `solver_cooldown_sec`,
- require a new full confirmation after cooldown.

Immediate release avoids executing stale opponent geometry. Delayed promotion
prevents one-frame prediction changes from repeatedly rebuilding a tight QP.

Initial values:

- confirmation: 0.20 s,
- solver cooldown: 0.50 s.

## Fail-operational policy

The first failed target-bound solve still uses the existing one-cycle
deceleration fallback. On the next 40 Hz cycle, only opponent hard bounds are
removed. Wall bounds and the currently planned lateral reference remain, so a
wall-only warm-start solve can recover before the existing three-cycle
Overtake Recovery trigger.

Failures that also occur with wall-only bounds continue through the existing
solver and Recovery policy; this change does not hide a wall or base-MPC
failure.

## Compatibility

- No topic, service, message, launch or result schema changes.
- Configuration keys are added to both local and cloud YAML files.
- The change remains inside the participant controller package.
