# Requirements: physical diagonal separation audit

## Objective

Replace the normalized candidate-E hypothesis with a shadow candidate whose
diagonal separating row is derived from the same ego footprint and opponent
circle geometry used by the exact nonlinear certificate.  Use the frozen
Domain 1 failure snapshot to decide whether a production repair is justified.

## Frozen evidence

- Source commit: `43daa303`
- Run: `output/20260828-094214`, Domain 1
- Decision: `1566`, Pass wall-refinement solve rejected
- Interaction fingerprint: `7246006054995400977`
- Snapshot: `000000001566-pass-wall-refinement-solve-rejected/snapshot.yaml`
- Candidate E: two certified left bundles, but 32 additional solved bundles
  were rejected by the exact dynamic proof.

## Invariants

- Production authority and live configuration remain unchanged during the
  physical candidate comparison.
- Clearance, solver tolerance, weights, horizon and Mission lifecycle are not
  tuned.
- The ego footprint and opponent radius are read from the immutable replay
  world captured with the frozen snapshot.
- Every numerical result must pass the unchanged exact swept wall proof,
  exact timed dynamic-obstacle proof and terminal successor proof.
- Old snapshots without the new optional physical-guidance data retain their
  interaction fingerprint.

## Acceptance gate

- The physical support row reduces to exact front-body separation at zero
  angle and exact selected-side body separation at ninety degrees.
- Intermediate rows use an oriented-rectangle-plus-circle support value, not
  normalized longitudinal/lateral radii.
- At least one candidate reproduces a complete certified bundle on the frozen
  failure snapshot.
- Solved candidates failing exact proof remain rejected and are reported.
- Only after this gate passes may a separate production Slice replace the
  axis-only producer and delete `partial_side_escape` atomically.

## Non-scope

- No production obstacle-policy change in this Slice.
- No additional fallback, resume rule, retry, timeout, lease or grace period.
- No parameter tuning.
- No claim of physical infeasibility from a failed local solve.
