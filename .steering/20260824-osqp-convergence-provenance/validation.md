# Validation

## Static validation

- Failure-first compile: the telemetry-contract test initially failed because
  `SolveTelemetry` did not expose convergence provenance.
- `make autoware-build`: 25 packages completed successfully.
- Package test from `/aichallenge/workspace`: 40 test programs, 1676 tests,
  zero errors, failures or skips.
- A first test invocation used the stale `/aichallenge/build` tree and produced
  two ABI/symbol lookup failures. Re-running against the actual workspace build
  passed completely; the stale-tree result is not product evidence.

## Dynamic observation

- Command: `make down`, `make dev2`, bounded observation, `make down`.
- Artifact: `output/20260824-022828`.
- The AWSIM admin start service was not discoverable, so no lap acceptance is
  claimed. Both domains nevertheless executed the closed-loop five-state
  Track/Cruise solve sufficiently to test the numerical contract.
- Domain 2: 31 certified and 34 execution-primal-rejected outcomes.
- OSQP success status coexisted with exact per-row physical violations.
- No decision predicate consumed the new telemetry.

## Acceptance result

The observation Slice is accepted. It distinguishes the hypotheses without
changing production behavior:

- warm transport is not the root producer;
- active bounds contribute to frequency;
- mixed-unit global convergence versus per-row physical certification is the
  structural mismatch;
- certification is incomplete for rate/dynamics rows even on some currently
  accepted outcomes.

The next implementation Slice must repair the five-state numerical coordinate
contract. It must not tune OSQP, loosen a physical bound, add a retry/fallback,
or suppress the downstream execution certificate.
