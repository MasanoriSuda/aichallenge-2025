# Requirements: Stop producer/consumer latency

## Problem

`output/20260831-134900/d1` had a certified current-world Stop plan before
decision 1563, but the retained consumer rejected it as
`steering-unreachable` when normal authority failed.  Existing telemetry logs
the consumer decision and plan sequence but not the producer decision or the
plan age at consumption.

## Objective

Join producer identity and consumer failure in one bounded telemetry record so
the next run can distinguish scheduling latency from a current-world model or
join defect.

## Constraints

- Observation only; no authority or candidate selection change.
- No timeout, lease, grace, fallback or parameter change.
- Preserve immutable plan identity.

## Definition of Done

- Alternate Stop telemetry reports consumer decision, source decision,
  sequence, snapshot age and control-origin age.
- Build and tests pass.
- A bounded dynamic run captures the first failed join.
- The causal classification is recorded before any production fix.

