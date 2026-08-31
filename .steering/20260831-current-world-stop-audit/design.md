# Design: current-world Stop audit

## Root cause under test

`compare_terminal_stop_lateral_contract()` currently solves the normal source
first and only then builds both seven-state Stop arms from the normal execution
artifact.  That is the publisher-boundary Stop construction, not the
current-world Stop construction used by the live independent worker.

When normal solve fails, both Stop arms report `candidate unavailable` without
testing any Stop control.  This makes an all-arms failure look like physical
infeasibility even though the Stop hypothesis was never evaluated.

## Change

Build the observation-only Stop audit from the immutable current-world source
using `build_current_world_maximum_braking_candidate()`.  Evaluate the direct
seven-state Stop and the shared steering-rate population from that candidate.
The comparison remains unable to publish or influence production.

## Invariants

- The source fingerprint and replay world are unchanged.
- The control prediction origin is not rebased.
- Exact wall and all-peer dynamic proof remain mandatory.
- Production selection and authority ledgers are untouched.

