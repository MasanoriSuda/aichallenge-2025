# Design

## Current authority path

```text
build legacy/three-state problem
  -> solve legacy or extended-overtake formulation
  -> convert five-state result to legacy vector when available
  -> otherwise solve three-state/legacy
  -> postprocess and publish

after production solve:
  -> solve Track/Cruise five-state shadow
  -> retain only one comparison ActuationProposal
  -> selected=0
```

The ordering means Track/Cruise shadow is observational, not an authority candidate. A certificate
alone is also insufficient: it does not contain the remaining input sequence required to continue
the same formulation after one failed solve.

## Pure selection contract

Add a small contract-level resolver independent of ROS, Eigen and the controller class.

```text
CanonicalNormalCandidate
  problem
  solution
  executable_control_stage_count

CanonicalNormalAuthorityRequest
  current_decision_id
  now_sec
  fresh_candidate
  retained_candidate

CanonicalNormalAuthorityResolution
  source = FreshCertified | RetainedCertified | EmergencyStop
  reason
  selected problem/solution identity
```

Candidate qualification requires:

- complete problem fingerprint;
- `VelocityProgress5State` formulation on both problem and solution;
- Track or Cruise intent;
- matching solution/problem fingerprint;
- full physical certificate;
- finite, unexpired validity;
- at least one executable control stage;
- current decision identity for the fresh candidate.

The retained candidate deliberately preserves its original problem identity. The final trace can
therefore state that a current decision published a retained plan without inventing a new solve.

## Why this is separate from warm start

A warm start is only an optimizer initial guess. It may be stale, uncertified or have no remaining
executable prefix. Treating it as fallback authority would repeat the existing provenance defect.
The later runtime store must retain both the certificate and the executable input/prediction plan.

## Promotion boundary

This Slice stops before wiring the resolution to `/control/command/control_cmd`. Runtime promotion
requires an explicit decision because it changes normal control authority. The following runtime
work is prepared but not authorized here:

1. build and certify Track/Cruise five-state before any legacy normal solve;
2. store its full input/prediction plan atomically;
3. advance and re-certify the retained prefix;
4. publish the resolver selection;
5. delete Track/Cruise legacy/three-state normal fallbacks.
