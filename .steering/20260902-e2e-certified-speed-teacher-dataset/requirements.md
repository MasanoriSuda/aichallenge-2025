# Requirements

## Objective

Turn the admitted seed-2033 `speed_committed_teacher` rollout into an
auditable offline dataset without changing production authority or silently
reusing the historical `precontact_teacher` provenance.

## Root cause

The new teacher depends on current wheel speed and encounter-local side state.
The existing relabeler replays LiDAR alone and the existing outcome certificate
validator accepts only `precontact_teacher`.  Adding the new mode name without
causal speed synchronization would therefore generate labels from a policy
that did not execute in the certified run.

## Constraints

- keep production v11, packaged artifacts and launch defaults unchanged;
- preserve historical teacher identities and certificates;
- accept only an executed, zero-penalty, zero-stall strict competition report;
- synchronize each LiDAR scan to the latest preceding wheel-speed sample;
- reject missing, future-only, stale or non-finite speed evidence;
- give the successor teacher distinct raw and recurrent label sources;
- preserve the historical precontact teacher as the paired diagnostic
  reference for successor-upgrade measurements;
- do not train or promote a model in this Slice.

## Definition of Done

- certificate validation is explicitly bound to the expected teacher mode;
- the relabeler reproduces the stateful teacher sequentially with causal speed;
- metadata records speed topic, type, timestamps, ages and teacher identity;
- existing historical datasets remain readable;
- tests cover causal matching, future-only rejection, provenance and
  certificate-mode mismatch;
- the admitted seed-2033 run is extracted into a new ignored immutable dataset.
