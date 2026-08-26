# Requirements

## Goal

Restore canonical normal publication when an accepted six-state execution
artifact contains a lower-bound residual that is already covered by its sealed
physical certificate.  Do not add another normal authority, fallback, timeout,
lease, feature flag, or parameter adjustment.

## Evidence boundary

- baseline HEAD: `005505e`
- moving run: `output/20260826-103853`
- domain: d1
- first post-start incident: decision 898
- transition: `Track -> Cruise`

At decision 898 the same immutable six-state plan reports:

```text
solver=solved
physical=accepted
world=accepted
joined=0
production_reason=command-rejected
```

## Required invariant

The production adapter must interpret a certified lower-bound residual with
the same sealed tolerance used by artifact and physical-trajectory validation.
The raw solver artifact remains immutable.  The physical command/horizon may
project a value inside that certificate to its exact physical boundary once,
before canonical command construction.  A value outside the certificate must
remain rejected.

## Constraints

- Preserve ROS, launch, Domain, result and submission contracts.
- Preserve the user-owned `aichallenge/result-summary.json` modification.
- Do not tune solver, wall, speed, acceleration, steering, horizon or timeout
  parameters.
- Do not weaken acceleration or steering-rate command bounds.
- Do not modify the raw execution artifact or exact physical proof.
- Do not promote another Overtake intent in this Slice.

## Definition of done

- A failure-first production-adapter test reproduces an accepted residual that
  is rejected at the command boundary before the fix.
- The certified physical projection accepts values inside the sealed tolerance
  and rejects values outside it.
- Canonical command and compatibility horizon contain the exact projected
  physical value and remain mutation-free at publication.
- Focused tests, full package tests, source contracts and workspace build pass.
- A moving run no longer reports `world=accepted, joined=0,
  production_reason=command-rejected` for this residual contract.
