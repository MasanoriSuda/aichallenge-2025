# Evidence

## Root-cause correction

The first production run, `output/20260825-230055`, formed complete Gate-A
proposals but the FSM rejected them as `identity-rejected`. The proposal had
been validated by the existing target-continuity contract, while the FSM then
required its source V2X observation generation to equal the newest live
generation. V2X normally advanced during the asynchronous solve, so the same
artifact was interpreted twice under different contracts.

The execution draft/result now transports the full `TargetProvenance`. The live
consumer invokes the existing continuity validator once and seals its source
generation into the Gate-A proposal. The FSM compares the CertifiedPlan with
that sealed proposal identity and does not compare the old source generation
directly with the newest observation. No timeout, generation grace, retained
proposal or relaxed threshold was added.

## Static validation

- `make autoware-build`: 25 packages passed.
- `multi_purpose_mpc_ros`: 49/49 test targets passed.
- Total package tests: 1875, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.
- The unrelated stale `build/joycon_contract_guard/package.xml` warning remains
  in `colcon test-result`; it did not change the successful test result.

## Dynamic validation

Run: `output/20260825-231050`

Domain 1 recorded:

- six-state ShiftOut Gate-A commits: 3;
- six-state ShiftOut atomic admissions with `certified=1`: 3;
- certified six-state ShiftOut final publications: 7;
- selected side coverage: `-1` and `+1`;
- five-state ShiftOut Gate-A acceptance: 0.

The first accepted chain was:

```text
target provenance source/live generation: 436 -> 438
Overtake entry commit accepted: gate=six-state-shiftout
Rate-resolved canonical atomic admission:
  previous=follow, intent=shiftout, solver=solved, physical=accepted
final execution contract:
  authority=certified-normal-solution
  formulation=velocity-steering-progress-6state
  intent=shiftout
  canonical_source=retained-certified
```

This proves that a causally certified six-state proposal can cross Gate A,
freeze the Mission geometry and publish through the same six-state normal
owner while V2X observation generation advances normally.

## Explicitly separate remaining evidence debt

This run did not reach Pass or Return. It recorded:

- 43 control callback overruns; whole-run callback maximum 57.394 ms against a
  25 ms period;
- one `ShiftOut -> Recovery` after `locked target stale or lost`;
- later `ShiftOut -> FollowPrepare` transitions from missing current-side
  prefixes or target/wall conflict.

Those failures occur after successful Gate-A admission and are not evidence to
restore five-state ShiftOut authority or relax target generation checks. They
are the next real-time/execution-lifecycle audit boundary. Direct Pass also
remains on its explicitly unpromoted five-state Gate A until exact Pass
proposal evidence is observed.
