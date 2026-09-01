# Requirements

## Objective

Obtain or reject the first strict outcome-certified rollout of the exact
`precontact_teacher` used by the current steering-label corpus.

## Constraints

- run the existing teacher unchanged in the deterministic one-ego/two-NPC gate;
- use an unseen explicit seed and preserve the complete output directory;
- require Finish, three laps, zero penalties and zero post-start stall;
- production v11, launch defaults, model artifacts and datasets remain frozen;
- a failed run is evidence against hard-label admission, not a trigger for a
  teacher threshold patch.

## Definition of Done

- the exact mode, world, seed and output identity are recorded;
- result-detail, motion and runtime logs are analyzed fail-closed;
- success opens a certified-data extraction slice;
- failure rejects current teacher output as an exclusive hard target.
