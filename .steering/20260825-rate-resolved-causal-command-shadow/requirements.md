# Requirements

## Purpose

Close the causal-lineage gap between the production five-state Track/Cruise
command and the six-state rate-resolved MPCC shadow pipeline before any
production-authority promotion.

## Root cause

The rate-resolved request is currently submitted while the synchronous
five-state Track/Cruise solve is still being evaluated.  The request therefore
captures `previous_steering` before the current production command is resolved.
The current cycle can subsequently publish a different steering angle, making
the otherwise valid asynchronous artifact unreachable from the command that
actually became steering history.

## Scope

- Seal the six-state request during the five-state evaluation, but do not submit
  it there.
- Bind state-zero steering and submit only after the current Track/Cruise output
  has updated the committed steering history.
- Convert an accepted retained proof into a typed six-state command candidate
  carrying complete identity and actuation provenance.
- Compare that candidate with the still-authoritative five-state command in
  shadow telemetry.
- Keep the six-state candidate disconnected from the publisher.

## Constraints

- No parameter tuning, new fallback, timeout, lease, or authority flag.
- No production-authority change in this Slice.
- Do not rebuild the five-state problem after command resolution.
- Do not label a six-state artifact as a five-state canonical command.
- Preserve the exact physical and dynamic-current-world evidence chain.
- Preserve all repository interface contracts.

## Definition of Done

- A source-contract test prevents rate-resolved submit inside the synchronous
  five-state evaluator and requires submit after command resolution.
- The submitted request's state-zero steering equals the output that became
  committed steering history for that cycle.
- A pure typed builder accepts only a complete retained proof and preserves
  decision, artifact, problem, geometry, intent, and actuation fields.
- Runtime logs report the six-state command candidate as
  `authority=shadow, selected=0` and show its delta to the five-state command.
- Package build and tests pass.
- A dynamic run confirms causal submissions and shadow candidates without a
  publisher or authority change.
