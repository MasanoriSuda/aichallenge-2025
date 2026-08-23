# Design: close architecture migration separately from solver quality

## Causal boundary

Track/Cruise production already has one five-state normal authority.  The
remaining failure chain is:

```text
mixed-unit five-state QP
-> OSQP solved/max-iteration result is unavailable at the strict execution boundary
-> retained current-world proof is unavailable in a one-car V2X NoData world
-> explicit canonical Emergency Stop
-> next certified five-state solution resumes normal authority
```

This is not a fallback to legacy MPC and is not a formulation handoff.  It is a
quantified availability defect inside the canonical formulation.

## Why another numerical patch is not part of this Slice

The following structurally distinct candidates were already implemented,
statically verified, dynamically falsified and removed:

- OSQP polish;
- downstream feasible-primal restoration;
- per-row QP normalization, including dual-coordinate rebase;
- physical-progress-coupled wall rows;
- scaled termination;
- full variable/constraint nondimensionalization;
- exact input-condensed QP reconstruction.

Each either retained the execution-boundary rejection or worsened physical
certificate, solve-time, maximum-iteration or callback-overrun behavior.  A
new tolerance, clamp, retry or fallback would repeat the patch cycle and
violate the single-authority migration policy.

## Closure rule

Slice 3 is complete when the architecture and six-lap integration gate pass,
even if the canonical solver sometimes becomes explicitly unavailable.  The
remaining numerical risk stays visible as Emergency authority and is deferred
to a future solver-backend/formulation investigation.  It must not be used to
justify keeping Track/Cruise legacy authority alive or blocking the audit-only
start of Slice 4.

Authority promotion for Follow/Hold/Stop remains a separate explicit boundary.
