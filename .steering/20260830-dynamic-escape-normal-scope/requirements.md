# Requirements: DynamicEscape normal-scope integration

## Frozen baseline

- Baseline commit: `d890c0a9`
- Dynamic run: `output/20260830-005711`, Domain 1
- Representative decision: 2725 near the first live pre-Mission DynamicEscape

## Observed causal sequence

1. GapPlanner exposes a validated pre-Mission DynamicEscape for target `d2`.
2. Canonical intent correctly resolves to `Cruise`; no fabricated Overtake
   execution identity exists.
3. The obsolete `mpcc_progress::resolve_activation()` branch nevertheless
   treats DynamicEscape as live progress execution.
4. Track/Cruise eligibility rejects the normal population as
   `live-progress-already-active`.
5. No Overtake execution identity exists, so the execution population is also
   unavailable.
6. The request reaches publication with `track=0/follow=0/execution=0/rejoin=0`
   and Emergency Stop is selected despite a valid normal avoidance intent.

The run contains 53 decisions in this exact family, beginning at decision
2725. This dynamically falsifies the previous assumption that deleting the
fabricated ShiftOut identity alone was sufficient to connect DynamicEscape to
the normal Track/Cruise producer.

## Root cause

DynamicEscape had two partially removed execution definitions. The canonical
intent/identity producer was demoted to normal Cruise in `bfaf7333`, but the
older progress-activation producer remained and still suppressed the very
normal producer now meant to own the action.

## Constraints

- Do not add a lease, grace, timeout, retry, resume rule or fallback.
- Do not change solver settings, weights, clearance or proof tolerance.
- Do not weaken current-world wall, obstacle, terminal-successor or Store
  publication proof.
- Do not publish GapPlanner geometry directly.
- A real ShiftOut/Pass/Return remains the only source of Overtake execution
  activation and identity.
- Remove the obsolete activation source in the same Slice which enables the
  normal scope; do not retain two routes.

## Definition of done

- DynamicEscape cannot activate the Overtake execution formulation.
- Cruise intent with DynamicEscape is eligible for the normal Track/Cruise
  current-world population.
- Its stage-wise lateral contract and current target tube remain attached to
  the normal seven-state request.
- Real Mission ShiftOut/Pass/Return behavior is unchanged.
- The old `track=0/follow=0/execution=0/rejoin=0` DynamicEscape signature is
  absent in dynamic acceptance.
- Focused tests, source contracts, package tests and build pass.

