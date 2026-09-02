# Requirements

## Objective

Replace the rejected in-process recurrent steering-authority route with an
explicit current-sample subprocess contract.  The recurrent candidate may
affect a command only when the separately isolated worker returns the exact
request sequence before that command is published.

## Root cause being addressed

The previous bounded-authority run evaluated the 512-hidden recurrent model in
the production callback process.  A 98.70 ms tail coincided with a lost speed
admission and hidden reset, so the strict single-vehicle Gate rejected it.
Authority-disabled process isolation now passes both single- and three-vehicle
Gates.  The remaining architecture defect is that explicit authority still
selects the old in-process evaluator.

## Constraints

- Production authority remains default-off and the recurrent artifact remains
  external to the submitted package.
- Do not publish a previous-scan, retained, latest-wins or otherwise delayed
  diagnostic correction.
- The child must verify exact artifact SHA/config and use one OpenBLAS worker.
- The response must match the current request sequence.
- The response deadline is the existing 50 ms watchdog/control period; a miss
  poisons that private channel and falls back to the already-valid spatial
  production command for the current sample.
- A recurrent worker failure may not stop the production controller.
- Do not change production checkpoints, acceleration, speed cap, steering
  bounds, obstacle thresholds, race scenario or Gate thresholds.
- Remove reachability of the old in-process recurrent-authority evaluator in
  the same Slice.

## Definition of Done

- Static tests prove authority cannot execute without an explicitly bound
  evaluator and that evaluator failure returns the spatial production command.
- Process protocol tests prove exact identity, sequence matching, one-thread
  resource policy, timeout poisoning and clean shutdown.
- Build and launch/interface tests pass.
- A single-vehicle authority A/B passes Finish, penalty, stall, scan-rate,
  coverage, reset and inference-error Gates.
- The peer Gate is run only after the single Gate passes.
- Packaged defaults remain recurrent-authority disabled regardless of the
  experiment result.
