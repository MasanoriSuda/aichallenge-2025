# Task list

- [x] Freeze the Return failure snapshot and production log interval.
- [x] Decode failed row 63.
- [x] Compare A--G arms on the same snapshot.
- [x] Compare horizon structure with `.steering/ano` and primary references.
- [x] Reconstruct and solve the recorded hard-constraint feasibility problem.
- [x] Determine whether exact physical proof can be reused for the independent
  feasible point.
- [x] Identify the earliest violated invariant.
- [x] Compare structural repair alternatives.
- [x] Implement only the selected root-cause repair.
- [x] Run focused tests and full package tests (49/49 CTest targets passed).
- [x] Preserve frozen replay as the immutable old four-stage failure oracle.
- [x] Confirm the common production builder emits 20 stages after the repair.
- [ ] Observe a clean dynamic `Pass -> Return -> Idle` completion (both trials
  failed upstream of Return and are classified as inconclusive).
- [x] Commit the root-cause repair and audit evidence as one Slice.
