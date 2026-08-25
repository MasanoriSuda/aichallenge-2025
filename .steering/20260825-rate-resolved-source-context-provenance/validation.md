# Validation

## Static and unit evidence

- `git diff --check`: pass.
- Focused rate-resolved tests: 6 tests, 0 failures.
- Full `multi_purpose_mpc_ros` package tests: 49 CTest targets,
  1,864 tests, 0 errors, 0 failures, 0 skipped.
- `make autoware-build`: 25 packages finished successfully.

The tests cover complete sealed source-context preservation, rejection of an
incomplete or five-state source context, and rejection of a different but
otherwise valid source identity at the certified-plan join.

## Dynamic evidence

Run: `output/20260825-091930`, short `make dev2` shadow check.

Both domains satisfied all of the following:

- every available rate-resolved command candidate reported
  `formulation:velocity-steering-progress-6state`;
- Track/Cruise artifact rejection and worker mailbox invalid counts remained
  zero;
- physical-wall `identity_mismatch` remained zero;
- control callback `overruns` remained zero;
- all rate-resolved results remained `authority=shadow, selected=0`;
- no fatal error or process crash was observed.

Dynamic-path blocking remained visible as a current-world retained rejection;
it was not hidden by the provenance refactor. The run was intentionally short
and is evidence for identity/runtime regression only, not a race-performance or
production-authority acceptance run.

## Scope confirmation

No parameter, timeout, fallback, margin, publisher authority or ROS interface
was changed. `aichallenge/result-summary.json` remained user-owned and was not
included in this Slice.
