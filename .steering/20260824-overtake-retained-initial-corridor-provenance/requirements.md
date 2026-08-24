# Requirements

## Objective

Identify the first canonical Overtake authority break observed in
`output/20260824-134024` without changing control behavior.

## Scope

- Preserve the exact `InitialCorridorViolation` operands as typed evidence.
- Emit measured/expected lateral position, time-zero corridor bounds, reserves,
  measured/expected progress and retained cursor stage in the canonical trace.
- Add deterministic tests for accepted and rejected provenance.

## Constraints

- No solver, wall, lateral or timing parameter changes.
- No grace, retry, lease, fallback or phase-transition change.
- No relaxation of current-world proof.
- Do not modify or commit `aichallenge/result-summary.json`.

## Definition of Done

- A replay or dynamic run distinguishes at least these hypotheses:
  1. measured plant pose left the sealed corridor;
  2. expected retained pose left its own sealed corridor;
  3. time/progress cursor selected the wrong corridor origin;
  4. the corridor itself was malformed or discontinuous.
- Existing tests/build pass and behavior remains unchanged.
