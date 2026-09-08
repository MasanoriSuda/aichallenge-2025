# Tasks

- [x] Trace actual V2X reference point through local scene and IL.
- [x] Compare complete body representations and required observations.
- [x] Seal rigid-body radius and a native physical-predicate overlap counterexample.
- [x] Establish failing producer regression and bounded implementation scope.
- [x] Repair and validate current-world/serialized-world geometry consistently.
  - Before-repair native overlap regression fails; new full-body fixture and native predicate pass.
  -25packages build;60CTestgroups/2338colconrecords,0errors/failures/skips.
- [x] Test initial multi-vehicle feasibility with the corrected complete envelope.
  - Firstdev2trial rejects: moving Emergency and actual Recovery; one lap each.
  - Same-source A solves but dynamic proof rejects; B/C/D/G lack a canonical target tube.
- [ ] Establish complete same-world peer constraints and compare candidate architectures.
- [ ] Resolve certified primal versus semantic physical initial-state boundary.
- [ ] Accept coupled race behavior and continue full integration/submission plan.

2026-09-09implementation continues under [repair-plan.md](repair-plan.md).
Baseline5cfc50bccontains the coordinate repair and initial single-vehicle evidence.
See[dev2-results.md](dev2-results.md)for the rejected trial and causal limits.
