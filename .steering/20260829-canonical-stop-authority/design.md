# Design

## Frozen evidence

Episode 2 in `output/20260829-155509/d1/autoware.log` admitted an exact,
physically certified right ShiftOut at decision 2111. Around decision 2136 the
retained trajectory became `steering-unreachable`; every current-world A/B/C/G
candidate in snapshot sequence 1501 then failed at QP row 295, the first-stage
steering-rate input bound.

The normal producer therefore emitted `canonical_emergency_stop`, but the
emergency retained the requested ShiftOut intent. The implementation only
applied the moving-Stop lateral policy when the requested intent was already
Stop, so it held approximately 0.269 rad steering while applying -3 m/s^2.
Forward progress collapsed while lateral motion continued. The next physical
check required 31--32 m/s^2 lateral acceleration and the actual footprint
violated the wall margin.

## Root cause

An emergency stop was represented as the failed normal intent instead of the
authority that actually owned the wire. This bypassed both the existing Stop
lateral policy and the atomic `Stop -> normal` admission bridge.

## Change

Make `canonical_normal_emergency_stop` internally canonicalize every requested
normal intent to external `ControlIntent::Stop`:

- record unresolved problem context as Stop;
- always resolve the speed-aware Stop lateral action;
- publish Stop as the authority identity;
- retain the failed requested intent only as diagnostic provenance;
- feed the actual Stop identity to the final execution trace.

No new control path is introduced. This removes the non-Stop constant-steering
emergency variant and reuses the single existing Stop supervisor.

## Why this is not a fallback patch

The wire was already commanded to stop. The change does not add another
fallback or decide when to stop; it makes that existing action obey one
authority contract. On the next cycle the existing atomic admission logic
keeps Stop until a certified normal artifact joins.
