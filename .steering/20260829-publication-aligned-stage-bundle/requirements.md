# Requirements: publication-aligned stage bundle

## Objective

Remove the normal-authority hole created when a seven-state control stage has
less than one publisher interval remaining.  The repair must rebuild and prove
one current-world Bundle from a command which can remain serialized for the
entire next publication interval.

## Frozen evidence

- Baseline commit: `29562adf`.
- Dynamic run: `output/20260829-220933`.
- D1 emitted 144 retained `continuation-rejected` transitions; 143 reported
  `continuation_model=invalid-cursor`.
- Representative failure: decision 5783, artifact 4433, published cursor
  0.200 s.  The current stage did not contain another 0.025 s publisher
  interval, so production emitted Emergency although the following stage and
  current world were available.

## Constraints

- Do not change wall clearance, solver tolerance, timing parameter, Mission
  lease, grace, retry, timeout or fallback behavior.
- Do not permit a short current stage command to cross the publisher.
- Do not mark an unmodified source artifact as executed when a stage boundary
  Bundle changes the command-time origin.
- Wall, dynamic-obstacle, Follow and terminal-successor proof remain mandatory.
- Final-stage exhaustion remains fail-closed.

## Definition of done

- An intermediate stage boundary selects the next complete command stage and
  replays it from the current physical state through the existing exact proof.
- The accepted proof is explicitly classified as a stateless current-world
  Bundle.
- A boundary with no next complete stage is still rejected.
- Source-contract, focused tests, full package tests and build pass.
- Dynamic A/B shows fewer `continuation_model=invalid-cursor` Emergency holes
  without a new wall/dynamic proof bypass.
