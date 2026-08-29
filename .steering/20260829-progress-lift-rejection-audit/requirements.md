# Requirements

## Objective

Explain and remove the `progress-lift-rejected` authority loss exposed by
`output/20260829-162420` without weakening retained-world proof or adding a
lease, grace period, timeout, fallback, solver tolerance, or clearance change.

## Frozen failure

- D1 episode 2, artifact sequence 27.
- Gate A admits ShiftOut at decision 1845.
- Retained proof first reports `progress-lift-rejected` at decision 1847,
  approximately 50 ms after first publication.
- Stop correctly owns the wire while normal authority is unavailable.
- The same artifact is briefly accepted at decision 1880 and rejected again
  at decision 1892 before its cursor is exhausted.

## Constraints

- Keep canonical Stop authority from commit `740fc273`.
- Do not tune the 1.5 m continuity tolerance until coordinate semantics are
  proved coherent.
- Do not make old Mission existence sufficient for publication.
- Preserve exact wall, dynamic-obstacle and terminal-successor proof.
- A behavior change requires a regression test that fails for the frozen
  causal mismatch.

## Acceptance

- The value compared by progress lifting has one documented physical meaning
  at both producer and consumer.
- A retained artifact whose current physical state remains joinable does not
  alternate Accepted/ProgressLiftRejected solely when the associated waypoint
  changes.
- A wrong lap or genuinely discontinuous physical progress remains rejected.
- Package build, all package tests and dynamic `make dev2` acceptance pass.
