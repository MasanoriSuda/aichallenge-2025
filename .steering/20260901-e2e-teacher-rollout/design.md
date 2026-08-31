# Design

## Root cause hypothesis

Round-one DAgger added only active labels from the student's failing trajectory.
Those labels describe local corrections, but not the successful state transition
created after the teacher takes the correction. Candidate2 improved offline on
that correction subset yet still entered an unrecoverable close-range state.

## Change

Use the admitted closed-loop teacher run as an additional train-only sequence:

```text
normal teacher rollout
  + failed-student pre-contact corrective labels
  + successful seed-2027 teacher rollout
  -> candidate3
```

The old independent teacher validation sequence remains unchanged. Seed 2027 is
only a reproduction gate after training; a different seed is required for
generalization acceptance.

## Non-changes

- No new heuristic runtime lateral owner.
- No production weight replacement before all gates pass.
- No duplication of student commands or contact suffixes.
