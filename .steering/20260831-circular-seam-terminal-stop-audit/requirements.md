# Requirements: circular-seam terminal Stop audit

## Objective

Explain why normal Cruise authority becomes `terminal-contingency-unavailable`
at the circular course seam in run `output/20260831-183224`, decision 4577,
without changing production authority or any solver, clearance, timeout,
lease, grace or fallback policy.

## Frozen evidence

- Decision 4574 still executes certified Cruise artifact 3083 at progress
  345.040 m.
- The reference crosses waypoint 347 to waypoint 1 immediately afterwards.
- Decision 4577 rejects the current-world physical snapshot as
  `terminal Stop course geometry unavailable` and publishes the already
  certified Stop successor from artifact 3083.
- Decision 4578 therefore begins with Stop authority while travelling at
  approximately 8.55 m/s.

## Definition of done

- The exact malformed geometry component and its size/provenance are visible
  in one causal rejection record.
- No production behavior changes in this audit Slice.
- Build and tests pass before another bounded dynamic run.
