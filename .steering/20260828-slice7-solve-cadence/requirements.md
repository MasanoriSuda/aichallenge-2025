# Requirements

## Objective

Complete the next evidence-driven Slice 7 experiment without weakening the
accepted 20-stage physical proof.  Separate the expensive asynchronous solve
cadence from the 40 Hz publication/current-world revalidation cadence.

## Constraints

- Keep `mpc.N=20` in local and cloud configurations.
- Do not change wall clearance, obstacle clearance, solver tolerance,
  authority, fallback, lease, grace period, or Mission lifecycle.
- Continue current-world revalidation and command publication at 40 Hz.
- Submit immediately when control intent, Mission generation, target, or
  execution side changes, or when current production authority is unavailable.
- Otherwise permit one fresh production solve every 0.05 s.
- Reject the experiment if an independent run loses the complete
  `ShiftOut -> Pass -> Return -> Idle` transition or adds Recovery/wall faults.

## Definition of Done

- Cadence decision is covered by unit tests.
- Build and single-authority contract tests pass.
- At least two independent dynamic runs are compared with the 20-stage 40 Hz
  baseline.
- The tuned value is either accepted with evidence or rolled back and recorded.

