# Design

## Root cause

The shadow Slice proved that the published Pass can lose its old cursor while
a freshly rebuilt maximum-braking successor is valid in the current world.
Production nevertheless jumps directly to the external Emergency owner because
the successor is only a diagnostic result and has no immutable artifact,
physical certificate or publication-ledger identity.

Returning its first speed/steering sample directly would hide the symptom but
would leave the next cycle without retained evidence.  It would also bypass the
single canonical normal publisher contract.

## Replacement

Introduce a Stop-successor bundle adapter which consumes the accepted
current-world request/result pair and constructs:

1. a current-decision seven-state execution artifact with the source target,
   intent and homotopy identity;
2. publisher-sized braking/path-tracking control stages and their exact state
   endpoints;
3. a physical snapshot containing the exact dense Stop trajectory;
4. a new accepted physical wall result from that exact snapshot;
5. an immutable certified plan.

The existing retained evaluator then rechecks the new plan against the current
dynamic world and the existing production adapter creates the canonical normal
command.  The final publisher records the plan only after exact serialization,
so subsequent cycles retain the Stop artifact instead of repeating the
authority gap.

## Authority rule

```text
ordinary normal authority available
  -> publish it

ordinary authority unavailable
  -> rebuild/prove current-world Stop successor
  -> reify certified plan
  -> current-world retained join
  -> canonical normal publisher

any successor or join failure
  -> existing external Emergency
```

There is no parallel normal owner.  Production selection still terminates in
the existing `CanonicalNormalCommand` boundary.
