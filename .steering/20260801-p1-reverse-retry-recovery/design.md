# Design

## Root Cause

The strict solver Reverse-only flag is stored as episode state, but its unlock policy also requires
the solver fallback to still be active. In the observed run, the solver recovered after creating
the episode, so a later completely blocked Reverse rollout could never unlock Forward.

Candidate course progress is checked before actuation, but there is no equivalent check against
measured motion. The observed Reverse steering produced the opposite lateral response from the
rollout prediction, allowing the kart to move progressively farther from the reference path.

## Changes

1. Treat `solver_reverse_only_episode` as the durable evidence for the unlock policy. At an
   aggressive retry, release it when Reverse candidates were checked and blocked while the current
   footprint, bounded Forward rollout, V2X corridor, and boost state are all confirmed safe. Do not
   require the transient solver-fallback signal to remain active.
2. Record lateral error at the start of each bounded recovery maneuver. During Reverse, detect
   measured worsening only when the vehicle is outside the rejoin envelope and absolute lateral
   error has increased by more than 0.10 m.
3. On measured worsening, clear the Reverse-only/intent latches, stop in Reverse, return to Drive,
   and reassess. Prefer the bounded Forward candidate for that reassessment; it still passes the
   existing footprint, course-progress, V2X, boost, and Drive-gear gates.

## Diagnostics

Add one transition warning when measured Reverse course progress worsens. Existing Recovery logs
continue to expose direction, lateral error, safety gates, and retry count.

## Scope

- `include/multi_purpose_mpc_ros/stuck_recovery_core.hpp`
- `src/stuck_recovery_core.cpp`
- `src/mpc_controller_cpp.cpp`
- `test/test_stuck_recovery_core.cpp`

No launch, parameter, topic, message, or evaluation-interface changes.
