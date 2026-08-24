# Requirements

## Purpose

Close the next observed single-authority migration gap without changing production control:
`OvertakeLinePhase::Recovery` resolves to canonical `ControlIntent::Rejoin`, but the five-state
canonical domain does not currently observe or certify that intent.

## Evidence boundary

- Root evidence: `output/20260824-085556/d1/autoware.log`.
- The episode entered `Recovery` after `actual footprint wall margin violated`.
- The joined final decision then reported `canonical_intent=rejoin` together with
  `formulation=legacy-spatial-mpc-3state` and `authority=legacy-normal-bypass`.
- This is a deterministic source-domain omission. It is not a parameter-tuning result.

## Scope

- Add a typed, shadow-only Rejoin eligibility boundary.
- Prepare five-state metadata from the existing Recovery line without activating the live
  progress-contouring/legacy conversion path.
- Reuse one canonical fresh evaluator implementation, but isolate Rejoin solver state, warm-start
  identity and plan storage from Track/Cruise.
- Admit `Rejoin` into the canonical artifact contract only so a complete shadow artifact can be
  constructed and audited.
- Emit Rejoin-specific shadow telemetry.

## Non-goals

- Do not publish a Rejoin canonical command.
- Do not delete the legacy Rejoin owner in this Slice.
- Do not add a timeout, fallback, feature flag or parameter adjustment.
- Do not reuse a Track/Cruise retained plan as Rejoin authority.
- Do not claim a production gate from static tests alone.

## Gate

Production promotion remains prohibited until a dynamic run exercises Rejoin and proves:

1. five-state solve and complete canonical artifact construction;
2. current-to-horizon bare-footprint wall clearance;
3. no cross-intent plan-store or warm-start reuse;
4. bounded runtime cost;
5. an explicit retained/current-world policy suitable for Rejoin.
