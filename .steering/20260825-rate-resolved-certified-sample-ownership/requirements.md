# Requirements

## Objective

Remove duplicate actuator-constraint ownership between the certified
rate-resolved QP and its 40 Hz actuation sampler. Restore one traceable
certificate boundary before any production-authority migration.

## Root cause

Run `output/20260825-005557` classified 515 sample rejects. Of those, 505
rechecked initial steering, steering rate or first-stage terminal steering at
an unrelated absolute `1e-12` tolerance after the QP had already passed its
row-scaled physical residual certificate. The sampler also used the solver's
numerical reconstruction of state zero instead of the immutable semantic
current steering.

This is duplicate constraint ownership, not QP infeasibility.

## Scope

- Define an explicit sample API whose precondition is a valid whole-QP
  physical constraint certificate.
- Use immutable semantic current steering as the integration origin.
- Keep rate and future stage-state bounds owned by the QP certificate.
- Keep publication-time ordering and the actual sampled steering/curvature
  fail-closed in the sampler.
- Preserve the strict standalone sampler for callers without a QP certificate.
- Extend deterministic tests and telemetry provenance.

## Non-scope

- No clamp, parameter, physical limit, solver tolerance or OSQP setting change.
- No production authority, fallback, publisher or Recovery change.
- No repair of publication intervals crossing a stage boundary; those ten
  typed rejects remain visible for the next independent Slice.

## Preserved user state

`aichallenge/result-summary.json` is a pre-existing user change and must not be
edited, staged or committed.

## Rollback

Rollback target: `c30a7d2`.
