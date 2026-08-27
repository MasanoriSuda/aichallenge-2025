# Requirements: Slice 6 executed-intent replenishment

## Purpose

Close the canonical normal-authority hole in which a physically certified
Overtake command remains executable, but the next asynchronous MPCC problem is
built for a transient Follow/Cruise proposal and therefore cannot replenish
the intent which still owns the publisher.

## Baseline evidence

The unmodified run `output/20260827-194608/d1/autoware.log` establishes the
causal sequence:

- decision 2921 publishes ShiftOut plan 2300;
- decisions 2922--2937 retain that executed ShiftOut after the tactical layer
  proposes Follow;
- those same cycles submit Follow problems rather than ShiftOut successors;
- at decision 2938 plan 2300 is exhausted and neither intent has current-world
  authority, so Emergency owns the command at about 6.0 m/s.

The rejected experiment in
`.steering/20260827-slice6-reachable-preview-horizon/` proved that merely
lengthening the artifact horizon does not restore this invariant.

## Required invariant

While the last successfully published canonical normal command is an
Overtake intent and its executed certified artifact still has an available
cursor at the current control origin, that executed identity must participate
in the next problem assembly when no newer live execution identity exists.

The retained identity must expire exactly with the artifact cursor. It must
not be renewed by age, a timeout, a lease, a flag, or tactical state alone.

## Constraints

- Keep the canonical seven-state rate-resolved MPCC as the sole normal owner.
- Keep Emergency and stuck Recovery external to normal authority.
- Do not change controller gains, wall margins, solver settings, horizon
  length, or V2X thresholds.
- Do not re-solve synchronously in the 40 Hz callback.
- Live OvertakeLine/DynamicEscape identities take precedence over retained
  executed identity.
- Retained identity is valid only when the executed plan, last-published
  intent, target, generation, side and artifact cursor form one exact identity.
- Once the cursor is unavailable, no retained identity may be synthesized.

## Acceptance

- Unit tests reproduce transient DynamicEscape -> retained ShiftOut -> cursor
  expiry without an indefinite identity latch.
- Source contract verifies that the canonical identity phase owns the
  authority request used to construct the MPCC problem.
- Package build and full package tests pass.
- Single-car dev preserves Track/Cruise operation.
- dev2 evidence shows that an executed ShiftOut retained across a transient
  Follow proposal receives same-intent successor submissions before cursor
  exhaustion, or records a typed current-world rejection rather than silently
  submitting the wrong intent.
