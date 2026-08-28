# Requirements

## Observed failure

In `output/20260828-200728`, domain 1 repeatedly produced physically certified
Track/Cruise artifacts but never published one.  The current-world join rejected
each new candidate as `steering-unreachable` and canonical Emergency kept the
last published steering command at zero.

The candidate store already distinguishes a certified candidate from the last
actually published plan.  The consumer did not: both were sampled at
`current_control_origin - artifact.prediction_origin`.  A candidate which had
never crossed the publisher was therefore advanced through a prefix which the
vehicle had not executed.

## Required behavior

- An unpublished candidate starts its execution clock at cursor zero.
- A published plan advances only from the control origin recorded when its
  first command crossed the publisher boundary.
- Candidate age alone must never imply executed controls.
- Both paths must still rebuild the exact continuation from the current state
  and pass current wall, dynamic-obstacle, Follow-gap and actuator proofs.
- Promotion and its execution-origin record must remain atomic at the exact
  serialized-command join.

## Prohibited shortcuts

- Do not enlarge steering reachability, add a grace/lease/timeout, or publish an
  unreachable command.
- Do not reset an already executed plan to cursor zero.
- Do not bypass current-world physical proof.
- Do not change solver tolerances, wall clearance or production authority.

## Definition of done

- Deterministic tests distinguish candidate and executed clocks.
- Store tests prove the publication control origin is recorded atomically.
- Full package build and tests pass.
- A bounded `make dev2` run shows domain 1 can promote a fresh normal artifact
  instead of self-sustaining `steering-unreachable -> Emergency`.
