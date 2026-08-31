# Tasklist

- [x] Existing E2E / NPC scenario and input contract audit
- [x] Paired deterministic teacher / student scenario design
- [x] Add scripts, Make targets, documentation, and static contract checks
- [x] Build / launch contract validation
- [x] Collect and inspect MPC teacher encounter (rejected by admission)
- [x] Refuse extraction from failed MPC peer runs
- [x] Run runtime NPC student baseline (stalled under positive acceleration)
- [x] Rename peer modes as audit-only to prevent false teacher provenance
- [x] Add machine-readable E2E run admission metrics
- [x] Record evidence for clean single and failed NPC baseline
- [x] Commit accepted tooling changes

Blocked by design, not deferred silently:

- NPC dataset extraction: no admitted avoidance teacher exists
- Candidate training: must follow a new obstacle-policy design Slice
- Closed-loop A/B: requires an admitted candidate first
