# Design

## Architecture question

The production arm currently consumes an unpublished asynchronous candidate at
the suffix aligned with the current control origin.  This fixes stale-time
comparison, but the skipped prefix has never been applied.  Therefore the
suffix may require a steering state that the physical actuator cannot have
reached.

The observation-only B arm evaluates the same candidate from artifact elapsed
time zero.  It is a diagnostic lower bound, not a production connector.  If B
passes where A fails, the artifact was physically joinable when produced and
became unjoinable only because the asynchronous prefix was skipped.

## Production invariant

`ExecutionClockKind::UnpublishedCandidateOriginComparison` is accepted only by
the retained proof evaluator.  The controller deliberately skips
`rate_resolved_command::build()` and `rate_resolved_production::build()` for
that clock.  The resulting observation is logged with:

```text
authority=observation-only, selected=0
```

No configuration flag or publisher branch can select it.

## Why this is not the final connector

Origin acceptance does not authorize replaying an old first command.  A valid
production design must instead use one of these structures:

1. AS-RTI style: asynchronous preparation about the predicted next state,
   followed by a fast feedback QP using the latest measured state.
2. ASAP-MPC style: begin the new trajectory on the currently executed
   trajectory and track it with a fast feedback controller.
3. Upper-log style: solve the main GMPCC from the current state at a bounded
   lower rate while asynchronous workers evaluate only tactical homotopies.

The comparison decides which design work is justified; it does not add another
Mission grace rule.

## Reference evidence

- acados AS-RTI example: preparation and feedback phases are separated, and
  the latest initial state is applied in the feedback phase.
- Nurkanovic et al., AS-RTI: the advanced problem is built from a predicted
  next state based on the last applied input.
- ASAP-MPC: asynchronous optimization is connected through an on-trajectory
  update and fast feedback tracking.
- `.steering/ano` upper logs: the main GMPCC is solved from the current state;
  asynchronous work evaluates tactical left/right candidates rather than
  splicing an independently evolving main control prefix.
