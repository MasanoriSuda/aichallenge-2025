# Requirements

## Objective

Evaluate the admitted peer512 recurrent artifact online around moving peer
vehicles without changing the published steering command.

## Constraints

- Run the deterministic three-vehicle peer scenario.
- Domains 1 and 2 retain MPC; only domain 3 runs the frozen E2E production
  controller plus recurrent shadow.
- Keep recurrent authority disabled and verify zero authority applications.
- Require exact checkpoint identity, at least 99% coverage, no inference errors,
  and bounded freshness resets.
- Evaluate Finish, penalties, stalls and runtime provenance before interpreting
  recurrent corrections.
- Do not package or promote the artifact from shadow evidence alone.

## Definition of Done

- The analyzer selects domain 3 explicitly rather than silently reading d1.
- Production race and motion results are frozen.
- Online recurrent coverage, timing, correction materiality and reset lifecycle
  are reported.
- The candidate is admitted or rejected for a later bounded-authority A/B.
