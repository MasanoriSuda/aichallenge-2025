# Requirements: DynamicEscape normal-authority integration

## Frozen baseline

- Baseline commit: `e3ac36e4`
- Dynamic run: `output/20260830-001650`, Domain 1
- Representative decisions: 1777--1815 near waypoint 113

## Observed causal sequence

1. Before an OvertakeLine Mission exists, GapPlanner exposes a validated
   DynamicEscape candidate for target `d2`.
2. The authority resolver promotes that tactical action to canonical
   `ShiftOut` and fabricates a ShiftOut execution identity from the escape
   attempt.
3. The normal worker already owns a certified Cruise dynamic-obstacle
   population, but its candidate and executed artifact are rejected as
   `intent-mismatch` against the fabricated ShiftOut intent.
4. Gate A is not attempted because no ShiftOut Mission has been admitted.
5. When the older Cruise continuation expires at decision 1803, neither the
   fabricated ShiftOut nor the discarded Cruise artifact can join, so normal
   authority becomes unavailable at 2.64 m/s and Emergency Stop is published.
6. The real `OvertakeLine: Idle -> ShiftOut` transition occurs later, after a
   Gate-A-certifiable Mission exists.

## Root cause

The 2026-08-25 compatibility producer which mapped a pre-Mission
DynamicEscape path to a canonical ShiftOut identity survived the later
introduction of the shared Cruise/Follow current-world obstacle population.
The old Mission-shaped producer and the new normal obstacle producer now own
the same encounter under different intents.

## Constraints

- Do not add a lease, grace, timeout, retry, resume rule or fallback.
- Do not change solver settings, weights, clearance or proof tolerance.
- Do not weaken exact current-world wall, obstacle or terminal proof.
- Do not allow a GapPlanner path to bypass canonical MPCC publication.
- Actual ShiftOut authority still requires an admitted OvertakeLine Mission
  and the existing Gate A identity/proof chain.
- Remove the obsolete producer in the same Slice which installs the normal
  semantic owner.

## Definition of done

- Pre-Mission DynamicEscape resolves to normal Track/Cruise intent, not
  ShiftOut.
- It is evaluated by the existing bounded two-side normal dynamic-obstacle
  population and keeps current-world proof requirements unchanged.
- DynamicEscape no longer manufactures an Overtake execution identity.
- Actual OvertakeLine ShiftOut/Pass/Return identity and Gate A are unchanged.
- Encounter homotopy ownership is keyed by the dynamic obstacle rather than a
  missing Mission target.
- Focused tests, source contracts, package tests and build pass.
- Dynamic acceptance observes no moving Emergency caused by
  pre-Mission DynamicEscape-to-ShiftOut intent mismatch.
