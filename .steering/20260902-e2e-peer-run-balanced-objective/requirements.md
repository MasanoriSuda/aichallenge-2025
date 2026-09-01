# Requirements

## Objective

Test whether one correlated four-domain peer run dominates recurrent training
and hides useful material steering corrections.

## Constraints

- Keep the 512-unit capacity candidate architecture and every loss/optimizer
  setting fixed.
- Change only successor sampling from natural chunk frequency to equal total
  mass per immutable executed-teacher `source_run_id`.
- Treat the four peer domains as one run, not four independent worlds.
- Require an executed outcome certificate and non-empty run identity for every
  sampled sequence; fail closed otherwise.
- Do not change runtime authority or packaged checkpoints.

## Definition of Done

- Unit tests prove equal mass per source run and rejection of missing
  provenance.
- Training manifest records sampling mode and per-run chunk counts.
- Seed 2033, unseen seed 2035 and production-normal comparisons are complete.
- Convert only if interaction material and aggregate behavior are not worse
  than the previous admitted candidate on both fixed worlds.
