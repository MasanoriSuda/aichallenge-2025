# Requirements

## Purpose

Prevent an MPCC trajectory that is wall-feasible but not continuously connected to the measured vehicle state from taking over overtake execution.

The `20260819-100123` run recorded execution ownership with `dp_bridge=1`, `prefix_promoted=0`, `stitch=0`, and `reachable=0/0`. The vehicle then diverged from the requested lateral path, the tracking MPC repeatedly reached the OSQP iteration limit, and stuck recovery was required.

## Scope

- Treat a solved MPCC source as a one-cycle bridge only after atomic handoff promotion has succeeded.
- Keep an already valid DP execution authority unchanged.
- Do not let wall validation alone prove measured-state continuity.
- Keep wall/contact/emergency hard faults authoritative.
- Add pure unit coverage for bridge admission.

## Constraints

- Do not change ROS topic, message, service, launch, or evaluation contracts.
- Do not change overtake aggressiveness parameters.
- Do not modify generated output or result files.
- Preserve the last-feasible source as a candidate; it must pass the same connected handoff before execution.

## Definition of Done

- A raw physically validated source without atomic promotion cannot own bridge execution.
- A physically validated, atomically promoted source can bridge the promotion cycle.
- Existing DP authority remains effective without requiring a solved-source bridge.
- Unit tests and the target package build/test pass.
