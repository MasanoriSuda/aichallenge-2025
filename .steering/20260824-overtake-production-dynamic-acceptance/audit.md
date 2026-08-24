# Audit

## Baseline evidence

The preceding accepted Gate is `output/20260824-081312`, Domain 1. It proved 21 `ShiftOut`
authority decisions: nine certified canonical five-state commands and twelve explicit Emergency
decisions, with zero legacy normal authority. `Pass` and `Return` had no positive execution coverage.

The same run left `ShiftOut` through `DynamicWait` into Recovery after exact physical proof could not
find a wall-feasible current-side authority. A later Track/Cruise solver failure occurred after
Recovery and is not part of this Overtake acceptance timeline.

## Current conclusion

The Gate is rejected. The promoted `ShiftOut` boundary itself did not fall back to a competing
normal formulation, but the same Overtake episode entered line `Recovery` and resolved the canonical
intent to `Rejoin`. `Rejoin` then fell through to the legacy three-state controller. This is a
normal-authority switch inside one Overtake episode and violates the repository's single-authority
target.

## Runtime findings

Evidence: `output/20260824-085556/d1/autoware.log`.

### Observed timeline

1. `Idle -> ShiftOut` at waypoint 258.
2. ShiftOut emitted three sampled certified canonical decisions and fifteen sampled explicit
   canonical Emergency decisions; sampled legacy ShiftOut decisions were zero.
3. A rolling DynamicWait was exercised once. It preserved `intent=shiftout` and emitted explicit
   Emergency, not legacy authority.
4. ShiftOut re-entered with a fresh same-side Mission, then exact runtime wall handling reported
   `actual footprint wall margin violated` at waypoint 266.
5. The next decision resolved `phase=Recovery`, `action=recovery`, `canonical_intent=rejoin`, but
   published `formulation=legacy-spatial-mpc-3state`, `authority=legacy-normal-bypass`.
   Three sampled Rejoin decisions carried that violation.
6. Pass and Return were not reached. Validated DynamicEscape was not exercised.

### Causal classification

- Root: the canonical intent resolver produces `Rejoin`, while the canonical normal intent domain,
  Overtake async intent set, five-state activation phase and retained Overtake proof domain exclude
  `Rejoin` explicitly.
- Trigger: ShiftOut lost its exact wall margin and requested the existing line Recovery.
- Contributor: canonical ShiftOut availability was intermittent, including current-world
  stage-corridor rejects and solver maximum-iteration outcomes.
- Mask: line Recovery appeared to restore motion, but only because the old three-state normal owner
  silently resumed control.
- Detection gap: none. The final joined trace names the problem, intent, formulation and violated
  authority boundary on one decision.

This is not a configuration or wall-clearance tuning issue. The legacy switch is deterministic for
every `Rejoin` intent because it is omitted from the canonical production domain by source.
