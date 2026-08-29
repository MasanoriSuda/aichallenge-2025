# Design

## Ownership

```text
raw async preparation
        |
        +-- joins serialized predecessor --> existing Store --> publisher
        |
        +-- steering-unreachable and no authority
              |
              +-- one-shot connector claim
              +-- bind after exact serialization
              +-- primary worker: common-clock suffix solve
              +-- unchanged physical/dynamic certification
              +-- existing Store --> existing retained evaluator --> publisher
```

There is one candidate authority ledger.  The connector is a producer of a
new immutable candidate, not an alternative consumer or publisher.

## Scheduling

The claim owner has `Idle` and `InFlight` states.  `InFlight` suppresses normal
worker submissions.  Completion or explicit submission failure returns the
owner to `Idle`; neither path schedules a retry.  The latest normal world is
still captured each callback so the first post-completion job is current.

This is load ownership, not a time-based rate limiter.  It ensures that full
solve and suffix solve do not compete and that a newer raw Store sequence
cannot overtake the connector before its result is consumed.

## Identity

The preparation's semantic context and homotopy remain immutable.  A new
canonical sequence identifies the newly solved suffix.  The output artifact,
physical snapshot/result and certified plan all carry that same sequence.
The source preparation sequence remains connector provenance and is recorded
separately in telemetry.

## Failure handling

- suffix/problem/solve rejection: complete claim, resume fresh normal jobs;
- physical or dynamic rejection: complete claim, resume fresh normal jobs;
- Store rejection: complete claim, retain the last actually executed plan;
- current-world join rejection: do not reconnect the connector result;
- explicit Emergency/Recovery behavior remains outside normal authority.

No branch changes a clearance, tolerance or vehicle command directly.
