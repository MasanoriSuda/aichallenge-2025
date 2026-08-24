# Audit

## Causal finding

The variable MPCC stage duration and the fixed 25 ms publisher period are two
valid clocks. The former certified sampler incorrectly required the publisher
boundary to remain inside stage zero, even though the solved QP already
contained a certified piecewise steering-rate sequence across the whole
horizon. The 11 runtime rejects were therefore caused by discarded certified
time-domain information, not by an infeasible actuator command.

## Responsibility repair

- The semantic current steering remains the only integration origin.
- The shadow evaluator extracts every solved steering-rate input together with
  its immutable stage duration.
- The certified sampler walks those stages to the exact publisher time.
- Every fully crossed steering endpoint and the final partial endpoint are
  checked against the physical steering box. Constant rate makes that endpoint
  check sufficient for each segment.
- Publication beyond the whole horizon has a distinct fail-closed reason.
- The obsolete certified single-stage API was removed rather than retained as
  a competing migration path.
- The strict non-QP single-stage sampler remains unchanged.

## Timing and authority audit

No stage duration, publisher period, horizon, solver setting or physical limit
changed. The new telemetry records the selected stage, elapsed time within it
and total certified horizon duration.

The result still terminates in observation-only aggregate telemetry with
`authority=shadow, selected=0`. It cannot enter any plan store, authority
selector, command history or publisher. All 24 single-authority source-contract
tests pass.

## Static conclusion

This is a time-base contract repair, not a timing relaxation or fallback. The
static Slice is eligible for committed-source `make dev2` validation. Normal
authority must remain blocked until that evidence confirms zero sample rejects.
