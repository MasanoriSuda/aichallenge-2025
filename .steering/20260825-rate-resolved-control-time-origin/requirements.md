# Requirements

## Purpose

Give the six-state rate-resolved Track/Cruise artifact, retained cursor, ego
delay prefix, and dynamic-obstacle prediction one explicit time contract before
production authority is considered.

## Root cause

The optimization state is initialized from the latency-predicted execution
pose, while the artifact currently labels its prediction origin with the raw
callback observation time.  Retained revalidation resolves the suffix cursor
at the callback time, joins it to the predicted execution pose, and checks the
measured-to-control path against dynamic obstacles at zero elapsed time.

With a constant delay, elapsed cursor arithmetic can appear correct because the
same offset cancels between cycles.  The physical meaning is nevertheless
ambiguous and the dynamic proof is wrong: the ego delay prefix spans future
time, but moving obstacles are frozen throughout it.  This is unsafe to promote
and cannot be repaired by age thresholds or clearance tuning.

## Scope

- Distinguish observation/capture time from control-effective prediction time.
- Store the control-effective origin in every six-state solver snapshot and
  execution artifact.
- Resolve retained cursors at the current control-effective time.
- Carry elapsed time for every measured-to-control ego pose.
- Evaluate moving obstacles through the delay prefix, same-time connector, and
  retained suffix on one continuous time axis.
- Keep all six-state commands observation-only.

## Constraints

- No parameter tuning, fallback, timeout, lease, or authority flag.
- No production publisher connection in this Slice.
- Do not alter five-state production ownership.
- Static wall and dynamic-obstacle evidence must share the same ego geometry.
- Existing ROS 2 and evaluation interfaces remain unchanged.

## Definition of Done

- Failure-first tests expose the old zero-time delay-prefix behavior.
- Artifacts distinguish capture time from control-effective prediction origin.
- Invalid or inconsistent time provenance fails closed.
- A moving obstacle which crosses only during the latency prefix is rejected.
- Build, package tests, and `make dev2` dynamic shadow validation pass.
