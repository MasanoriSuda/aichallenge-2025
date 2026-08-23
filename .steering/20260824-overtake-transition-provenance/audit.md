# Audit

## Hypotheses

### H1: missing homotopy identity is the structural root cause

- Support: early ShiftOut replanning mutates `pass_side_sign` without changing
  Mission generation; the canonical context contains intent/generation/target
  but not side.
- Refutation: prove that every possible side mutation also changes intent or
  generation before canonical construction.
- Confidence: high. A direct same-phase mutation exists in
  `replan_early_shiftout_side()`.

### H2: stored-plan evaluation corrupts failure attribution

- Support: `evaluate_overtake_async_shadow()` passes the same mutable result to
  incoming evaluation, then to stored evaluation when incoming selection is
  incomplete. Telemetry later maps any complete result to `fresh-selected`.
- Refutation: prove the stored pointer is always null/equal to incoming at every
  transition.
- Confidence: confirmed by code flow; runtime source counts are unavailable.

### H3: current corridor is simply too strict

- Support: physical outcomes include initial/stage corridor violations.
- Refutation: classify failures by semantic artifact/source and show they occur
  on an old side/generation before changing any clearance.
- Confidence: unproven and deliberately not acted upon.

## Classification

- Root cause: selected homotopy is absent from canonical identity/lifecycle.
- Contributor: a plan store survives async context reset.
- Mask: late current-world physical rejection makes a semantic mismatch look
  like a corridor feasibility problem.
- Detection gap: incoming and stored evaluations share one mutable result and
  one outcome counter.

## Implementation audit

- The canonical problem fingerprint now includes `execution_side_sign`.
- ShiftOut, Pass and Return require exactly `-1` or `+1`; Track, Cruise and
  Follow require `0`.
- A same-phase side change advances the asynchronous context epoch, clears the
  old-family plan store and resets the mailbox.
- An invalid Overtake semantic context also invalidates the family instead of
  merely skipping one submission while retaining an old plan.
- Retained semantic identity is checked before course-frame, wall and dynamic
  corridor proof. Cross-side rejection is therefore typed as
  `execution-side-mismatch`.
- Incoming and stored plans are evaluated into separate immutable diagnostics;
  only the selected result becomes the aggregate command candidate.

No clearance, solver, timeout, lease, fallback, authority or ROS-interface
change was made.

## Dynamic audit

`make dev2` produced `output/20260824-051821`. The approximately 231.5 second
log contained:

- Overtake state transitions: 0;
- Overtake canonical shadow evaluations: 0;
- Follow `stage-gap-violation` log lines: 1,247;
- Follow `target-horizon-unavailable` log lines: 60;
- Follow emergency-authority traces: 652;
- Follow retained current-world accepted traces: 1,041;
- callback runtime windows with a non-zero overrun count: 2 (three reported
  overruns in total);
- `execution-side-mismatch`: 0, as no Overtake context was entered.

The run therefore cannot prove the side-transition acceptance criterion. It
does prove that the new Overtake path did not execute and that the blocking
failure is upstream: Follow current-world proof alternates between accepted and
terminal-stage gap rejection while the measured front gap is roughly 14 m.
That issue must be investigated in a separate failure-first Slice. It is not a
reason to loosen Overtake corridor parameters or promote Overtake authority.
