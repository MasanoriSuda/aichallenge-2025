# Requirements: Return lifecycle escape-hatch audit

## Objective

Classify the actual-wall failure after a completed Pass without changing
production authority.  Determine whether the failure belongs to persistent
Mission lifecycle, Return candidate generation, single-SQP approximation,
physical infeasibility, or model/certificate mismatch.

## Frozen evidence

- Baseline: `5c435a7b`.
- Dynamic run: `output/20260831-015209`.
- Failure snapshot: decision 2856, intent `Return`, side positive,
  `terminal-contingency-unavailable`.
- The retained artifact was about 1.60 s into execution while actual pose and
  velocity had diverged from its expected prefix.
- The current comparison tool rejects Return C/D arms as unsupported, so an
  all-failed result cannot yet be classified as physical infeasibility.

## Constraints

- Do not change production authority.
- Do not add Mission resume rules, leases, grace periods, timeouts, fallbacks,
  solver tolerances, clearance changes, or parameter tuning.
- Compare A/B/C/D from the same immutable world/problem fingerprint.
- A successful solve is not sufficient: exact wall/opponent proof, terminal
  successor viability and contingency Stop suffix are mandatory.
- If evidence is incomplete, classify `Unknown`; do not infer physical
  infeasibility.

## Definition of done

- Return snapshots admit meaningful A/B/C/D comparison rather than reporting
  unsupported C/D arms.
- A deterministic test fixes the Return comparison semantics.
- The frozen decision-2856 snapshot is classified with evidence.
- Only after classification may a separate production slice be designed.
