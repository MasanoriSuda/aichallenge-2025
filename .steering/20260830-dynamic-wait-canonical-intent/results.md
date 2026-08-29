# Results: DynamicWait canonical-intent handoff

## Root-cause conclusion

The first DynamicWait Emergency was not evidence that both sides were
physically blocked. The canonical execution identity already preserved the
interrupted ShiftOut Mission, but the canonical intent resolver required an
optional legacy DynamicWaitPrefix to be the lateral command owner. The prefix
was generated later in the cycle, so update ordering changed safety semantics.

The fix makes DynamicWait a tactical no-transition. A coherent canonical
execution identity preserves ShiftOut/Pass semantics; the ordinary
current-world candidate, seven-state solve and exact proof pipeline decides
whether a command may publish. The optional prefix remains reference
provenance and has no command ownership.

## Static verification

- source-contract tests: 77 passed;
- package CTest: 54 targets, all passed;
- `make autoware-build`: 25 packages passed;
- `git diff --check`: passed.

One failure-first CTest initially found that the new tactical homotopy owner
was assigned only inside the `line_active` branch. Moving the identity-owned
case to the top-level owner selection fixed the same structural scope error;
the focused test and all package tests then passed. No runtime fallback or
configuration was added.

## Dynamic comparison

Baseline: `output/20260830-004030`, commit `bfaf7333` parent behavior.

- four DynamicWait entries and five logged DynamicWait decisions;
- one decision used
  `canonical_intent=unknown/dynamic-wait-without-lateral-authority` and
  published Emergency Stop at 4.72 m/s;
- no DynamicWait decision used canonical-execution intent provenance.

Candidate: `output/20260830-005711`.

- three DynamicWait entries and four logged DynamicWait decisions;
- zero old Unknown-intent signatures;
- all four decisions resolved to
  `shiftout/canonical-execution-dynamic-wait-shiftout`;
- decisions 1664 and 2960 published certified retained ShiftOut solutions;
- decision 1664 remained at 4.40 m/s and commanded 4.48 m/s with
  `authority=certified-normal-solution`;
- decisions 2974 and 2975 still failed current-world admission and correctly
  used Emergency Stop. Their semantic intent is now coherent; their physical
  or terminal proof failure is a separate family and was not bypassed.

Total Emergency counts are not compared because the runs differ in length and
encounters. This Slice accepts the disappearance of the obsolete Unknown
intent and the observed certified publication on the repaired DynamicWait
path, while retaining fail-closed behavior when current-world proof is absent.

## Residual failures

- a separate DynamicWait encounter at waypoint 225 had coherent ShiftOut
  intent but no admitted current-world solution;
- an earlier encounter entered hard static-wall Recovery;
- ShiftOut/Pass terminal successor and progress-lift failures from the prior
  run remain separate root-cause candidates;
- callback and whole-race quality are not accepted by this Slice.
