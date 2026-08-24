# Design

## Typed authority resolution

Add a pure resolver to the Overtake core. Its inputs describe:

- whether canonical Overtake owns the current intent;
- whether the legacy reference/corridor artifact is structurally complete;
- whether an independent hard supervisor condition is active.

It returns one of two actions:

- `CanonicalReferenceOnly`: preserve the Mission and pass the reference plus
  hard corridor to canonical MPCC; the legacy physical result is telemetry
  only;
- `LegacyFailClosed`: retain the existing transition handling.

This is an authority decision, not a timeout or fallback.

## Runtime behavior

For `CanonicalReferenceOnly`:

1. retain phase, target, side and Mission generation;
2. do not arm retry-block, DynamicWait or Recovery from the legacy rejection;
3. do not apply the legacy recovery-speed cap;
4. preserve the generated lateral reference and hard stage wall corridor;
5. let canonical current-world proof either publish a certified command or
   select the existing explicit Emergency supervisor.

The next callback receives the same Mission identity, so a newly completed
async canonical plan can be evaluated against the current world rather than
being rejected because the Mission generation was destroyed.

## Rejected alternatives

- Relaxing 0.40 m wall clearance or 6.0 m/s2 lateral acceleration: hides the
  ownership defect and changes racing parameters.
- Holding the previous command for a fixed time: creates another lease.
- Treating an old retained canonical plan as valid without current-world
  proof: violates the canonical certificate contract.
- Deleting the receding reference builder immediately: leaves the current
  five-state problem without its Overtake reference/corridor input.
