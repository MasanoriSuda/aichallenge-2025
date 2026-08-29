# Requirements: Follow stateless authority deletion

## Root cause

The same-world architecture comparison for
`output/20260829-103359/d1/...sequence 471` reproduces the earlier frozen
Follow classification:

- persistent Mission Follow A: solver rejected after 4000 iterations and
  114.567 ms;
- stateless current-world left B: fully certified in 51.613 ms;
- stateless current-world right B: fully certified in 53.081 ms.

Production still evaluates A before B. A therefore consumes the freshness
budget before a feasible B candidate is available. This is the previously
classified Mission lifecycle defect, not a solver-parameter or clearance
defect.

## Objective

Delete persistent Mission geometry evaluation from the production Follow
worker. Evaluate the bounded current-world Follow homotopy directly and admit
it only after the existing solver, exact wall and exact current-world dynamic
proofs accept the same artifact.

## Constraints

- Do not change solver settings, horizon, tolerance, clearance, timeout,
  lease, grace or fallback policy.
- Keep one canonical seven-state MPCC normal authority.
- Retain only target/intent identity and the selected Follow homotopy.
- Do not introduce another Store, publisher or command source.
- Remove the persistent Follow evaluation branch in this Slice.

## Definition of done

- The Follow population has no persistent Mission solve.
- Its first solve is the selected/current-world side candidate.
- The exact current-world dynamic certificate is mandatory for Store admission.
- Source-contract tests prevent restoration of `persistent-follow`.
- Frozen A/B comparison, focused tests, build and dynamic Gate are recorded.
