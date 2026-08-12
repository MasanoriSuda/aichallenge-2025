# Design

## Policy alignment

The existing `resolve_recoverable_side_contact()` already classifies a narrow
competition-simulation exception using Pass phase, target continuity, side
geometry, closing speed, lateral velocity, ego speed, elapsed time, and fresh
forward progress. The committed-corridor front-danger policy already consumes
that result, but committed Pass Behavior ownership does not.

Add `recoverable_side_contact_active` to the extracted Pass geometry ownership
request. When true it provides:

- Pass authority during the bounded contact, even if ordinary lateral/front
  cap latches are temporarily unavailable.
- Acceptable current geometry despite confirmed overlap.

The final Pass owner still requires a committed Pass, validated fixed line,
valid side, and all shared target/hard-fault guards. This prevents the contact
exception from bypassing map, emergency, target, or solver failures.

## Release behavior

When the recoverable-contact resolver becomes inactive because of excessive
closing speed, lateral closing, low ego speed, timeout, or stale progress, the
geometry owner immediately returns to the ordinary latch/handoff rules. A
confirmed overlap then releases Behavior ownership as before.

## Logging and configuration

Reuse the existing `OvertakeLine ContactContinuation entered/ended` log and
existing thresholds. No new runtime parameter or high-rate log is added.

