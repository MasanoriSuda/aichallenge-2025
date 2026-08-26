# Requirements

## Objective

Prevent an unproved normal-intent proposal from replacing the currently
executed six-state normal authority.  A Follow -> ShiftOut (or any other normal
intent transition) becomes effective only when the exact new six-state plan is
solved, physically certified and joined against the current world.

## Constraints

- Keep one six-state normal authority; do not restore five-state or legacy MPC.
- Do not add a timeout, lease, retry flag, clamp or parameter change.
- Emergency remains mandatory when neither the proposal nor the previously
  published six-state intent has current-world proof.
- Update the published intent only after the selected command crosses the ROS
  publisher boundary.
- Preserve the user-owned `aichallenge/result-summary.json`.

## Exit criteria

- A rejected ShiftOut proposal can publish a currently valid previous-intent
  six-state plan without claiming ShiftOut authority.
- An accepted proposal atomically publishes the new intent.
- No normal command is emitted without current-world revalidation.
- Dynamic run shows physically rejected ShiftOut proposals without the former
  matching Emergency pulses.
- A valid current Follow problem must consume the same continuity-constrained
  target observation that formed its six-state horizon; retained-proof
  checking must not independently select another course branch.
