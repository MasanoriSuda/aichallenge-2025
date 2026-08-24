# Design

## Earliest violated invariant

Selected solution, execution artifact, physical certificate, retained proof,
and command must share one formulation and problem fingerprint. The existing
source context violates this before the six-state worker starts.

## Producer repair

Introduce `VelocitySteeringProgress6State` in the shared execution contract.
`make_problem_context()` assigns its own state, input, bounds, and cost schema.
The Track/Cruise builder creates two contexts only while migration shadowing is
active:

- five-state context for the current production owner;
- six-state context for the rate-resolved worker draft.

The artifact identity carries the six-state formulation. Physical and retained
proofs already copy and compare the complete artifact identity, so adding the
field extends their existing all-or-nothing join rather than adding a branch.

## Mask removed

`mpcc_rate_resolved_command_candidate` has a private formulation enum whose only
value is six-state. It currently asserts a label that is absent from the
artifact. Remove it and expose the certified artifact formulation instead.

## Alternatives rejected

- Keep the five-state fingerprint and infer six-state from the C++ type: this
  leaves logs and final authority unable to prove the formulation.
- Change only telemetry text: this hides the mismatch.
- Promote now and repair identity later: this violates the production
  authority gate and preserves two conflicting normal contracts.

## Deletion milestone

This is the last identity-only shadow Slice. Once dynamic evidence confirms the
new identity, the next Slice must connect the six-state Track/Cruise command and
delete the five-state Track/Cruise normal owner atomically.
