# Task list

- [x] Inventory post-fix frozen snapshots by failure family and intent.
- [x] Replay representative progress-wall failures.
- [x] Replay representative dynamic-obstacle failures.
- [x] Replay representative terminal-contingency failures.
- [x] Separate production failures from observation-only audit failures.
- [x] Decode the first reproducible upstream infeasibility.
- [x] Record A/B/C/D classification and rejected hypotheses.
- [x] Select one root-cause production Slice or stop for missing evidence.
- [x] Defer canonical integration documentation until a production contract changes.
- [x] Commit only audit-owned files.

## Definition of Done

- At least one dominant live failure has a reproducible upstream cause.
- The next change is justified by a frozen counterexample, not a threshold.
- No production authority or parameter changes are included in this Slice.

## Evidence summary

- Candidate run: `output/20260831-051051`.
- Snapshot inventory: 33 total; dynamic-obstacle refinement 10, terminal
  contingency 7, initial solve 5, successive linearization 6, wall-related 4,
  with two post-refinement physical/dynamic SQP snapshots explicitly
  observation-only (families overlap by naming).
- ShiftOut snapshot 2230: persistent selected side failed; opposite stateless
  and production-direct side produced a certified Bundle.  At the same time
  production remained Cruise/Follow and the pre-entry telemetry was
  `authority=shadow, selected=0`; this is candidate-side evidence, not a live
  sibling-adoption failure.
- ShiftOut snapshots 3742 and 4709: neither direct side produced a Bundle;
  exact wall or exact dynamic proof rejected the solved alternate.
- Cruise wall snapshot 2684: opposite stateless side produced a Bundle and the
  live pipeline recorded `store=accepted/adopted_side=1`.
- Cruise terminal snapshot 1128: no arm produced a Bundle; exact dynamic proof
  found new d2 overlap (minimum clearances approximately -5.3 mm, -1.7 mm and
  -6.8 mm for persistent/left/right direct arms).
- Canonical production intent counts contained Track, Cruise and Follow only.
  ShiftOut/Pass/Return count was zero for d1, d2 and d3 despite tactical
  `Follow -> Overtake` messages.

## Rejected next changes

- Do not retain an older Overtake bank epoch merely because snapshot 2230 has
  a feasible opposite side; it was not active execution.
- Do not relax dynamic clearance for snapshot 1128; exact interpolated overlap
  is the reason every available direct arm failed.
- Do not change the wall bucket or normal sibling implementation for snapshot
  2684; live adoption already localized that failure.

## Selected next Slice

Trace the entry authority chain:

`tactical Follow -> Overtake` -> mission identity -> pre-entry certified Bundle
-> Gate A -> canonical ShiftOut -> publisher.

The first lost identity/authority edge, rather than the later shadow solver
failure, is the next production root-cause target.
