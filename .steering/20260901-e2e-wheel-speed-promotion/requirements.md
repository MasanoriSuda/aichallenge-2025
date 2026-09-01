# Requirements

## Objective

Promote the qualified spatial steering adapter without violating the End to
End input contract or relying on repository-local launch overrides.

## Root cause

The qualified DAgger v3 adapter consumed speed derived from
`/localization/kinematic_state`.  Although only longitudinal speed was read,
that topic is fused localization and is outside the intended E2E input set.
The permitted vehicle input is wheel odometry through
`/vehicle/status/velocity_status`.

## Requirements

- build training data and runtime inference from the same wheel-speed topic;
- exclude the held-out failed authority sequence from training by immutable
  sequence identity;
- preserve the admitted base model and the `0.12 rad` authority bound;
- require two reproducible NPC seeds before promotion;
- package the candidate below `aichallenge_submit/`;
- verify the packaged candidate SHA256 before loading;
- keep exactly one learned steering owner;
- provide an explicit rollback to base-only steering;
- do not depend on `Makefile`, `.env`, `aichallenge_system`, or the ML
  workspace in the submitted archive.

## Definition of Done

- unit and package contract tests pass;
- `make autoware-build` passes;
- single and NPC closed-loop gates pass without penalties or stalls;
- launch defaults select the packaged artifact in a sealed submission;
- the submission archive has one `aichallenge_submit/` top-level directory
  and contains the exact qualified artifact;
- production defaults are verified in a closed-loop run.
