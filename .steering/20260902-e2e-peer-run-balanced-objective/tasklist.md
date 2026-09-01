# Tasklist

- [x] Freeze source-run balancing hypothesis and acceptance boundary.
- [x] Implement fail-closed outcome-run balanced sampling and manifest evidence.
- [x] Add focused and full workspace tests.
- [x] Train the otherwise-identical 512-unit candidate.
- [x] Evaluate seed 2033, unseen seed 2035 and production-normal behavior.
- [x] Reject conversion/runtime-shadow admission; authority remains unchanged.

## Decision

Equal source-run mass does not recover interaction quality.  Material MAE is
worse than the previous candidate on both fixed worlds, and is also worse than
the naturally sampled 512-unit candidate.  Stop sampling-ratio tuning in this
Slice.
