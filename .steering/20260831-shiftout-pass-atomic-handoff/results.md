# Results: atomic ShiftOut-to-Pass handoff

## Static verification

- `make autoware-build`: passed, 25 packages built.
- Package CTest: 59 / 59 passed.
- `git diff --check`: passed.

The regression contract verifies that tactical horizons alone cannot authorize
the transition, a proposal with non-Pass intent is rejected, and the exact
target, Mission generation and side must match before phase mutation.

## Dynamic verification

Run: `output/20260831-081212`, bounded `make dev3`.

Domain 2 episode 3 produced a current-world Pass proposal at sequence 205 and
then performed the transition at decision 3379:

```text
OvertakeLine: ShiftOut -> Pass ...
reason=shift complete ... with atomic current-world Pass authority
Rate-resolved canonical atomic admission:
previous=shiftout proposed=pass effective=pass
resolution=proposed-accepted gate_a_attempted=1 gate_a_joined=1
```

The command published in the same decision had `phase=Pass`, `intent=pass`,
`solver=canonical-rate-resolved-pass-certified-candidate`, and solution/plan
identity 211. The old failure signature -- tactical `phase=Pass` while a
retained `intent=shiftout` artifact remained the normal owner -- did not occur
at this transition.

When no current-world Pass proposal was available, domain 1 remained in
`ShiftOut` and logged `reason=proposal-incomplete`; it did not mutate the phase
without authority. This is the intended fail-closed outcome and does not add a
time-based permission.

## Separate remaining failure

About 1.15 seconds after the successful atomic handoff, domain 2 entered
Recovery for `actual footprint wall margin violated`. This is not evidence
against the handoff repair: Pass authority was already exact and identity
complete at decision 3379. It is frozen as the next independent execution
tracking / wall-certificate audit and must not be hidden by a clearance,
tolerance, timeout or fallback change in this Slice.
