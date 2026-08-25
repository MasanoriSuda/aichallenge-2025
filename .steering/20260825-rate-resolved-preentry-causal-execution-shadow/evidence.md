# Evidence

## Static validation

- `make autoware-build`: 25 packages passed.
- Focused source contract: 54 passed.
- Full package CTest: 49/49 test targets passed, 1889 assertions, zero
  errors/failures/skips.
- The source contract proves that the causal execution worker binds its draft
  only after the current normal command is committed, uses a private mailbox
  and null production store, and can only perform a shadow current-world join.

## Dynamic observation

### Rejected callback build

- `output/20260825-213018` proved that a complete causal selected-side build is
  possible, but placed 32--43 ms of prospective-problem construction in the
  25 ms callback and caused repeated callback overruns.
- That implementation was rejected. The callback now only freezes a deep-owned
  immutable snapshot; problem construction and solve run in one latest-only
  worker.

### Corrected worker boundary

- `output/20260825-214120` observed no Overtake candidate and no callback
  overrun; callback maximum was 15.557 ms.
- `output/20260825-214344` exercised selected-side execution. Snapshot copying
  cost 0.094--0.477 ms while worker-side builds cost 31.937--44.975 ms. At
  least five jobs completed solver and physical proof, and no callback overrun
  occurred in the exercised candidate window.
- Every completed result reported `identity_current=0`. Timeline inspection at
  1787661866.461--1787661866.885 showed why: the selected left homotopy was
  followed by a new tactical `selection-unavailable` result before the causal
  worker completed. The old combined identity gate therefore suppressed
  current-world proof and could not tell “physically unsafe” from “no current
  tactical authority”.
- The gate is now typed into exact/newer-same-side/selection-unavailable and
  explicit contradiction outcomes. A selection-unavailable result may only be
  physically revalidated in shadow; it cannot become authority-ready.

### Typed identity and current-world outcome

- `output/20260825-215909` exercised the corrected worker boundary and typed
  live identity join.
- Domain 1 published two latest-only results. Both completed the six-state
  solve and exact physical proof:
  - tactical sequence 48: snapshot 0.355 ms, worker build 95.592 ms,
    total worker 104.622 ms;
  - tactical sequence 50: snapshot 0.190 ms, worker build 35.630 ms,
    total worker 38.477 ms.
- In both cases the live tactical selection had already become unavailable.
  The typed contract therefore permitted the physical current-world
  observation but kept `tactical_authority_current=false`.
- Both current-world joins then independently rejected the completed plan as
  `steering-unreachable`. The totals were complete=2, world-join=0 and
  authority-ready=0. This distinguishes two separate facts which the former
  combined boolean hid: the tactical intent had expired, and the current
  Follow actuation had diverged from the asynchronously solved prefix.
- During the candidate/worker interval the 40 Hz callback stayed within its
  25 ms budget: the enclosing one-second windows reported maxima of 9.470 ms
  and 7.035 ms. The later continuous 32--57 ms overruns began after
  DynamicEscape production admission at 1787662796.264, more than four
  seconds after the last shadow result. They were charged to the production
  `mpc` region, not to snapshot construction; the execution shadow had no
  further published result in that interval.

## Verdict

PASS for a causal, observation-only selected-homotopy execution producer and
for the typed separation of physical observation from tactical authority.
This Slice does **not** authorize production. The dynamic evidence identifies
the next structural gate: an Overtake intent must remain current through the
execution solve and its prefix must join the actuation actually published
while that solve was running. Relaxing steering reachability or keeping an
expired selection would hide that failed handoff and is prohibited.
