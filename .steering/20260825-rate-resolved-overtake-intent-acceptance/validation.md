# Validation

## Observed failure before the correction

Run: `output/20260825-161843`

The tactical Overtake branch was selected, but the normal intent repeatedly
failed before reaching the shared six-state producer:

```text
Overtake entry commit rejected: source=preentry-gate-a ...
admission=steering-continuity-rejected ... action=keep-cruise-follow
```

The rejection came from first-actuation reachability of the selected five-state
tactical artifact.  This was the earliest rejected invariant and established a
dual-authority defect: the retired five-state representation still gated live
steering before the six-state atomic transition admission.

## Failure-first test

Added
`test_five_state_preentry_artifact_cannot_gate_rate_resolved_actuation`.
Before the implementation it failed because the resolver still called
`extract_canonical_actuation`.  It now requires that the tactical resolver
validate cursor/identity only and that fresh entry not install the artifact in
the old production plan store.

The C++ contract test was changed from expecting unreachable five-state
steering to be rejected to proving that the tactical resolver does not own live
steering authority.

## Static verification

- `make autoware-build`: passed, 25 packages.
- Package tests: 49 of 49 targets passed.
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  1827 tests, 0 errors, 0 failures, 0 skipped.

## Dynamic verification after the correction

Run: `output/20260825-163126`

- old `preentry-gate-a ... steering-continuity-rejected`: 0
- six-state ShiftOut atomic admissions certified: 3
- retained six-state ShiftOut publications observed: yes
- published formulation:
  `velocity-steering-progress-6state`
- stale/cross-intent/uncertified normal publication observed: no
- Pass/Return reached: no

The retained publication joined the same certified identity:

```text
intent=shiftout
formulation=velocity-steering-progress-6state
canonical_source=retained-certified
retained=1
identity=complete
canonical=satisfied
reason=matching-certified-solution
```

## Remaining blocker classification

There were 62 rejected six-state ShiftOut atomic admissions:

- 53 `stage-wall-rejected` with `rate-resolved stage hard wall contact`;
- 9 `semantic-steering-sequence-rejected`.

In addition, tactical search frequently reported no complete or receding branch
candidate and `ShiftOut/Pass path requires wall clamp`.  These are explicit
solver/physical/path-quality rejections, not recurrence of the removed
five-state authority gate.  They are not bypassed or tuned in this Slice.

Pass and Return therefore remain a dynamic-evidence debt.  Their absence is now
explained by an upstream path-quality blocker rather than an untraceable owner
handoff.  The next Slice must audit that blocker or delete the remaining retired
Overtake lifecycle; it must not restore five-state command authority or relax
wall proof.
