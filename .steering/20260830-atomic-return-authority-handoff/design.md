# Design

## Causal transition pipeline

The existing pre-entry latest-only worker becomes an intent-transition worker
for two mutually exclusive jobs:

1. Mission Gate A for ShiftOut/Pass entry or replacement;
2. Return Gate A when a live Pass has a valid geometric Return reference.

Only one job is submitted per callback. Return has priority after rear-clear
because the live controller must finish the already committed encounter before
evaluating another lateral Mission.

## Prospective Return problem

The worker owns a deep tactical snapshot. On that private snapshot it changes
the phase from Pass to Return, keeps the existing Mission generation/target/
side, consumes the already computed Return preflight reference, and builds the
same canonical seven-state problem used after a live Return transition.

The live `OvertakeLineState` is not mutated. The artifact passes the normal
SQP, nonlinear, swept-wall, dynamic-obstacle, terminal Return and contingency
proof pipeline.

## Return Gate-A identity

The immutable proposal contains:

- worker sequence and context epoch;
- Mission generation;
- target observation generation and target id;
- selected side;
- certified Return plan.

Consumption revalidates it against the current world. Tactical transition
admission then checks exact target/generation/side/intent identity. Geometric
preflight without this proposal remains Pass and keeps current Pass authority.

## Atomic publisher bridge

When the phase changes, canonical atomic admission evaluates the same Return
proposal before considering old Pass retention. A successful proposal is
published through the existing single canonical publisher and only then
becomes retained normal evidence.

No alternate publisher, timeout or fallback is introduced.

