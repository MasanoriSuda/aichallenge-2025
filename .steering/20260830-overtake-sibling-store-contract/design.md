# Design

## One Store sibling contract

Replace the Cruise/Follow-only helper with a typed sibling relation covering
the two existing producer families.

For normal avoidance:

- intent is Cruise or Follow;
- `dynamic_obstacle_side_sign` is opposite;
- the context is otherwise identical after resealing.

For active Overtake:

- intent is ShiftOut or Pass;
- both `execution_side_sign` and `dynamic_obstacle_side_sign` are opposite;
- the context is otherwise identical after resealing.

Sequence and snapshot time must be exact in both families. Return and all
other intents remain single-plan Store entries.

## Lifecycle

The same validator is used by candidate replacement, exact publication,
published-bundle recording and associated-sibling lookup. This avoids another
producer/consumer split and ensures that a pair admitted as a candidate can
survive through the publisher ledger unchanged.

No new fallback is introduced. A valid current Pass solution replaces the old
candidate; an invalid pair remains rejected.
