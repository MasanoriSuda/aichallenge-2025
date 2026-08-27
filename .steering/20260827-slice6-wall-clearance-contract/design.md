# Design

## Earliest violated invariant

Planning and physical certification must be produced together but retain
their distinct meanings.  All physical consumers must share the same
`physical_clearance_m`; planning/admission must retain
`required_clearance_m`.

## Causal chain

```text
wall contract producer resolves physical and required clearances
  -> the canonical problem retains only required clearance in active Mission scope
  -> pre-entry proposal has no physical value before Mission admission
  -> active current/stage physical envelopes are queried with 0.0 m
  -> immutable physical snapshot stores hard_clearance=0.0 m
  -> QP refinement uses the raw footprint
  -> fresh certificate proves only the raw footprint
  -> pre-entry certification is rejected or execution consumers disagree
  -> Emergency / Recovery / visible deceleration
```

The visible Recovery is downstream safety behavior, not the root cause.

## Repair

1. Resolve `WallClearanceContract` for every canonical problem, including a
   hypothetical pre-entry problem before a Mission becomes active.
2. Preserve both `physical_clearance_m` and `required_clearance_m` on
   `MpcProblem`.
3. Define one checked footprint expansion helper in the physical-wall module.
4. Give QP physical refinement, fresh proof, and retained proof the same
   laterally expanded physical footprint.
5. Build generic current/future physical wall envelopes with the physical
   clearance.  Keep the Overtake stage planning profile on its existing
   required-clearance contract.
6. Reject a physical snapshot when its physical clearance is missing or
   invalid.

No new production branch or configuration is added.

## Alternatives rejected

- Reuse required planning clearance as the physical footprint: dynamically
  refuted by `20260827-182448`; it over-constrains ordinary Track/Cruise.
- Increase wall margin: changes the number but keeps the ownership mismatch.
- Add a retained grace period: masks an under-certified fresh solution.
- Disable runtime wall rejection: permits a physically unsupported command.
