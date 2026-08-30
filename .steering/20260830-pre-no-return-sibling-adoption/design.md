# Design

## Root cause

Run `output/20260830-120323`, source sequence 2100, proved that the selected
side failed while the opposite side was exactly certified from the same world
more than three seconds before tactical abandonment. Production retained
consumption inspected sibling branches only for Cruise/Follow, so the active
Overtake sibling had no authority path.

## Authority flow

```text
same-epoch active branch bank
  -> selected branch has no current-world authority
  -> opposite plan revalidated against current world
  -> pre-no-return adoption contract
  -> canonical normal command
  -> serialized command identity join
  -> tactical homotopy commit
  -> retire old frozen Mission geometry
```

The adoption token contains source epoch, intent, target, generation, and old
and new sides. It is immutable between current-world proof and publication.
The final commit rechecks the live tactical identity; it does not use elapsed
time as authority.

## State ownership after adoption

Persistent state keeps the encounter target, Mission generation, current
phase, no-return state, progress budgets, and selected side. The previous
Mission plan, fixed lateral goal, Frenet path samples, outer-transition path,
and Return preflight path are cleared because they describe the rejected
homotopy. The published certified Bundle and subsequent current-world
receding solves own execution geometry.

## Non-goals

- rescuing epochs where neither side certifies;
- changing Return authority continuity;
- changing candidate population or numerical settings;
- changing emergency or Recovery policy.
