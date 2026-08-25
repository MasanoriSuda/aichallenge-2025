# Audit

## Classification

| Surface | Reachability | Responsibility | Decision |
|---|---:|---|---|
| left/right isolated five-state branch solve | live | tactical pre-entry Gate A | keep until six-state Gate A replacement |
| `canonical_normal_control()` | 0 callers | retired five-state command publisher | delete |
| `evaluate_overtake_async_shadow()` | 0 callers | retired retained selector root | delete |
| async Overtake mailbox/worker/status | dead-root only | retired transport | delete |
| five-state retained current-world evaluator | dead-root only | retired normal selector | delete |
| retained five-state plan store | dead selector + diagnostics | reconnectable retired authority state | delete |
| five-state Emergency problem context | live | false trace identity; no solve occurs | use `Unresolved` |
| five-state plan/adapter types | live | tactical Gate A representation | retain temporarily |

## Why this precedes Gate A migration

Removing the live pre-entry proof without a replacement would violate the
project rule that dynamic evidence must exist before authority promotion.
Removing unreachable publisher infrastructure first is behavior-preserving,
makes later review tractable and leaves exactly one explicit five-state
responsibility to replace.
