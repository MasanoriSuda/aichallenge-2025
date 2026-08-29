# Requirements

## Objective

Repair the retained-authority proof unit so it matches the command publisher's
causal unit.  A normal command is serialized for one publication interval; a
partial proof must therefore certify exactly that interval plus the already
required terminal Stop suffix, not an arbitrary remaining solver stage.

## Frozen failures

Baseline commit: `8dc45378`.

Run: `output/20260829-172407`, D1.

- decision 1464, sequence 858, stage 0, cursor 0.180 s;
- decision 2928, sequence 1968, stage 0, cursor 0.115 s.

In both events progress, steering reachability and velocity reachability are
valid.  The nonlinear continuation first leaves the lateral bounds later in
the still-active solver stage, so `build_continuation()` produces no partial
proof and canonical authority falls immediately to Emergency Stop.

## Repaired invariant

The minimum normal authority certificate is:

1. the exact current serialized command for one publisher interval; and
2. a current-world-certified terminal Stop suffix beginning with that same
   unavoidable publisher interval.

Solver-stage duration is planning discretization and may not define the
minimum publication authority interval.

## Constraints

- Do not change wall clearance, opponent clearance, solver tolerance, weights,
  rates or timing parameters.
- Do not add a lease, grace period, retry, resume rule or fallback controller.
- If the exact publisher interval itself violates a physical bound, reject it.
- A publisher-only prefix never receives authority without a certified
  terminal Stop suffix.
- Preserve one seven-state normal authority and the external Emergency Stop.

## Acceptance

- A deterministic case where a long solver stage leaves its corridor after
  the publisher interval is admitted only as a publisher-interval prefix.
- A case leaving the corridor inside the publisher interval remains rejected.
- Static-wall and dynamic-obstacle partial proof scopes use the same publisher
  interval, not the remaining solver stage.
- Build, focused tests, complete package tests and dynamic `make dev2` pass.
