# Design

## Root cause boundary

`evaluate_rate_resolved_normal_avoidance_population()` and
`evaluate_rate_resolved_normal_population()` sometimes construct a rejected
`Result` directly instead of returning the result of the solver pipeline.
Those paths set identity, outcome and detail, but omit the completion time and
compute duration required by `mpcc_rate_resolved_shadow::result_valid()`.

The worker then publishes the structurally incomplete rejection.  The mailbox
correctly rejects it as `invalid-result`, but this also removes the reason that
would identify whether both sides were physically impossible or candidate
construction itself failed.

## Change

Introduce one failure-result constructor at the worker pipeline boundary.  It
creates a complete, immutable, non-solved result with:

- the unchanged source identity;
- the explicit failure outcome and detail;
- a finite completion timestamp at or after the snapshot;
- the measured non-negative compute duration.

Use it for every early rejection currently assembled by hand:

- normal avoidance candidate population rejection;
- missing physical snapshot for normal avoidance;
- missing physical snapshot for an Overtake execution population.

No result contains an execution artifact, no certified Store entry is made,
and no authority path is changed.

## Dynamic evidence

After the change, rerun `make dev2`.  Existing telemetry must report the last
failure including its detailed population/candidate rejection.  That evidence
will determine the next behavioral Slice; this Slice deliberately does not
guess or tune around it.

