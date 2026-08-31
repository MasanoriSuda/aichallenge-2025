# Design

## Root cause

The sibling publication code correctly retires all geometry belonging to the
rejected Mission. Behavior ownership had two independent persistent-Mission
dependencies:

1. `mission_path_frozen && fixed_pass_corridor_goal_ey`; and
2. the admission-time `mission_body_clear_deadline_checked` / handoff state.

The first static change generalized only dependency 1. Dynamic run
`output/20260831-114002/d1` refuted that as a complete fix: sibling sequence
798 was published and ShiftOut commands continued, but Behavior repeatedly
demoted to Follow with `shift_owner=0`. No body-clear handoff entered after
the stateless adoption. The old Mission deadline therefore remained a hidden
ownership prerequisite after all geometry belonging to that Mission had been
retired.

The source-specific correction exposed one final duplicate owner in
`output/20260831-115922/d1`: sequence 598 was committed by the canonical
publisher and every logged hard guard passed, but Behavior re-consulted the
separate generation-only publication ledger. That ledger reported false while
the exact sibling publication identity remained active, so one publication
was effectively required to cross two independent lifecycle owners.

The resulting contradictory cycle is:

```text
stateless Bundle certified and published
  -> tactical side changes
  -> rejected Mission geometry is deleted
  -> Behavior still requires the retired Mission body-clear handoff
  -> Behavior demotes Overtake to Follow
  -> OvertakeLine/canonical publisher still execute ShiftOut
```

## Corrected ownership contract

Behavior must distinguish two execution-source kinds rather than collapse
them into one boolean:

1. a frozen Mission owns execution only while its admission-time body-clear
   deadline and handoff remain active; or
2. a stateless sibling Bundle owns execution after its exact serialized
   command crossed the canonical publisher. Its path and certificates were
   rebuilt from the current world, so it must not depend on a deadline owned
   by geometry that was deliberately retired.

The second source is represented by immutable `{Mission generation, source
sequence}` written only during exact publication-token adoption. It does not
rely on the legacy generation-only publication ledger. The identity is cleared
when ShiftOut/Pass ends or a new fixed Mission is frozen.

Both facts only preserve Behavior/phase ownership. They do not supply a path
or command. Every command continues through fresh/retained current-world
proof, exact wall/dynamic proof and the canonical publisher.

## Rejected alternatives

- Keep the old Mission geometry after sibling adoption: restores the rejected
  homotopy and recreates split ownership.
- Treat any worker result as Behavior authority: bypasses publication identity.
- Add a short Follow/Overtake grace or resume edge: hides the owner mismatch.
- Copy or extend the old body-clear deadline after side adoption: retains
  lifecycle state whose geometry and homotopy no longer exist.
- Relax the wall-clamp candidate check: B already succeeds with unchanged
  physical proof, so tuning is not causal.
