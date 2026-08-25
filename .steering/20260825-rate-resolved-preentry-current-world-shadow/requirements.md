# Requirements

## Objective

Prove that the immutable six-state branch selected by the prospective Gate A
can still join the live vehicle state and current dynamic world at the exact
Mission-adoption boundary.

## Requirements

- Reuse the production six-state current-world revalidator; do not introduce a
  second wall, obstacle or actuation proof.
- Verify selected side, six-state intent and target provenance before invoking
  current-world proof.
- Run only on the live controller side, after the async worker result is
  imported.
- Keep the result observation-only: it must not admit/reject the production
  five-state Mission, replace a production plan store or publish a command.
- Do not add a fallback, flag, timeout, lease or parameter change.
- Preserve ROS 2 and evaluation contracts.

## Definition of Done

- Common current-world request construction is shared with production normal
  authority rather than copied.
- Failure-first tests protect target/side identity and observation-only use.
- A bounded `make dev2` run records adoption attempts and rejection reasons.
- Build and package tests pass.
