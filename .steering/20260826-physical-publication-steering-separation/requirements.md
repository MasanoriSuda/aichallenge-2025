# Requirements

## Baseline

- HEAD: `9d8bfc2`
- Dynamic evidence: `output/20260826-113945`
- Primary examples: domain 1 decisions 1195, 2293, 2461 and 3504

## Objective

Remove the remaining dual meaning of the six-state steering origin.  A physical
steering state at the latency-compensated prediction origin must not also act
as the predecessor of the desired command publication sequence.

## Constraints

- Keep one six-state normal authority.
- Do not relax retained current-world revalidation.
- Do not add a clamp, timeout, lease, feature flag or legacy normal fallback.
- Do not tune steering rate, horizon, wall margin or solver settings.
- Preserve the user-owned `aichallenge/result-summary.json` change.

## Exit gate

- Physical predicted states retain the observed/projected physical origin.
- Desired steering publication retains the exact previously published command
  as a separately sealed predecessor.
- Artifact cursor samples desired publication from that predecessor and the
  exact certified steering-rate sequence.
- Failure-first, focused and complete package tests pass.
- Moving `make dev2` evidence shows the former high-rate reacquisition failures
  are removed without `command-rejected` or callback overruns.
