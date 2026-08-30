# Tasklist

- [x] Freeze baseline and dynamic failure run.
- [x] Reproduce both Return-to-wall episodes.
- [x] Compare early and late frozen snapshots across A--D arms.
- [x] Trace supervisor Return transition, generic stateless candidate builder,
  dynamic obstacle rows, Store and terminal successor proof.
- [x] Separate the physical opposite-side witness from valid Return semantics.
- [x] Add failing tests for Return reference rewriting and missing terminal
  semantic proof.
- [x] Add immutable current-world Return relation classification.
- [x] Add dedicated Return candidate population.
- [x] Add solved-terminal Return viability proof.
- [x] Replace the generic pass-style Return producer atomically.
- [x] Add decision and architecture-comparison diagnostics.
- [x] Remove accidental whole-file formatting while preserving the semantic
  Return changes. Restored the 15 affected files to `831232dc` after explicit
  approval, then reapplied only the reviewed semantic delta.
- [x] Run focused Return tests: 5/5 passed on 2026-08-30.
- [x] Restore `test_single_authority_source_contract`: passed after the source
  layout restoration.
- [x] Run complete package CTest: 55/55 targets passed on 2026-08-30.
- [x] Run `make autoware-build`: 25 packages passed after clean reapplication
  on 2026-08-30.
- [x] Run dynamic `make dev2` trial from the rebuilt artifact. Two independent
  episodes completed `ShiftOut -> Pass -> Return -> Idle`; the frozen Return
  rejection did not recur.
- [x] Separate episode 2 (`FollowPrepare -> Recovery`, static-wall admission)
  from the Return failure family instead of adding a mixed-purpose patch.
- [x] Compare runtime behavior with `.steering/ano` without tuning parameters.
  The existing lap ledger remains the performance baseline; this short causal
  trial does not replace a six-lap acceptance.
- [x] Record results and stage only intended files.
