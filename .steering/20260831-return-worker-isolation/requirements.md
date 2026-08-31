# Requirements: Return worker isolation

## Root cause

The instrumented run `output/20260831-145722/d1` proved that a 2.824-second
`MissionGateA` job blocked Return Gate A on the shared non-cancelable worker.
The latest Return request did not start until after Pass had lost authority
and the vehicle had entered wall-contact Recovery.

## Objective

Give Return Gate A an independent bounded execution lane so Mission/Pass work
cannot head-of-line block terminal successor generation.

## Constraints

- Keep the same Return candidate builder, seven-state formulation, physical
  proof, mailbox identity and consumer admission.
- Do not change production authority, timeout, lease, fallback, solver
  tolerance or clearance.
- Use a private solver context per concurrent worker lane.

## Definition of Done

- Return jobs use a dedicated latest-only worker and solver context.
- Mission/Pass jobs remain on the existing worker.
- Both lanes publish through the same immutable sequence/mailbox contract.
- Architecture contract test prevents lane re-merging.
- Build and tests pass.
- Dynamic run shows the first Return job starts without waiting for an older
  Mission/Pass job, then classifies its physical result.
