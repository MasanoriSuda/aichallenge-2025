# Design

## Causal hypothesis

The normal worker solves from a latency-compensated snapshot while the vehicle
continues to execute the previously published artifact.  Candidate selection
currently reports only the top-level candidate reason after falling back to the
executed artifact.  It discards the candidate's detailed continuation and
actuator evidence, so `continuation-rejected` cannot be assigned to a concrete
contract violation.

The first change is therefore diagnostic, not behavioral: preserve and emit
the complete candidate join classification alongside the executed selection.

After dynamic evidence identifies the failed contract, compare these structural
repairs:

1. Solve from a causally projected future execution origin.
2. Splice a fresh candidate at a reachable suffix and certify the connector.
3. Rebuild a stateless current-world bundle when an old snapshot cannot be
   connected.

An elapsed-time skip alone is not sufficient: controls in an unpublished
prefix were never applied.  Any suffix adoption must prove the current physical
state, published command origin, walls, and dynamic obstacles.

## Acceptance

- Candidate rejection logs identify the exact failed connector contract.
- No production behavior changes in the diagnostic commit.
- The subsequent structural fix has a regression test reproducing motion during
  asynchronous solve time.
- A moving vehicle can adopt successive fresh candidates without first stopping.
