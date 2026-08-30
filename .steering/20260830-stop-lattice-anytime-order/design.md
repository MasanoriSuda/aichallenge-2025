# Design

## Root cause

The live failure is not absence of a physically certified Stop candidate.
Accepted candidates were observed after 1, 8, 29 and 42 attempts.  The stale
tail is caused by population order: switch schedules are locally clustered and
all positive-rate schedules precede all negative-rate schedules.  A latest-only
worker can replace pending epochs, but it cannot cancel an already running
sign-major exhaustive solve.

## Change

Keep `build_population()` as the legacy audit order.  Add
`build_anytime_population()` with the same members and these ordering rules:

1. Build the legacy population once.
2. Group schedules by `(first_switch_stage, second_switch_stage)`.
3. Select the first geometry nearest the existing nominal horizon fractions
   `(0.15, 0.30)`.
4. Select each later geometry by deterministic farthest-point traversal in
   normalized horizon coordinates.  This makes every early prefix cover the
   schedule space rather than one adjacent cluster.
5. Emit the two initial steering-rate signs consecutively for each geometry.
   Prefer continuity with the normal publisher-boundary steering-rate; when it
   is zero, prefer the sign that unwinds current steering; use positive as the
   final deterministic tie-break.

The evaluator still returns the first candidate that passes the unchanged
solve, exact-trajectory, rest, wall, all-peer and certified-plan chain.  No
candidate is accepted on ranking alone.

## Comparison evidence

For the selected certified candidate, record:

- anytime rank;
- rank of the identical schedule in legacy sign-major order;
- total population size;
- preferred initial-rate sign.

This compares search scheduling without running a second full solver and
without doubling CPU load.

## Non-goals

- No production promotion.
- No search deadline or candidate-count cap.
- No warm-start sharing or solver setting change.
- No clearance or speed tuning.
