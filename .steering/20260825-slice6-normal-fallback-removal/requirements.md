# Requirements

## Purpose

Remove the remaining reachable legacy/three-state normal solve boundary from
`MPC::get_control()`. Every resolved supervisor intent must use its canonical
MPCC owner or explicit Emergency; configuration or admission failure must not
silently change formulation.

## Root cause

Canonical Track/Cruise, Follow, Overtake, Stop and Rejoin branches all return
before the old solve block during the accepted configuration. Nevertheless the
old extended/three-state/legacy solver remains as the lexical fallthrough for
an unresolved intent or a disabled migration prerequisite. This preserves the
exact dual-authority mechanism Slice 6 is intended to remove.

## Required invariants

- Track/Cruise routes by intent to the six-state owner. Missing eligibility or
  proof returns canonical Emergency, never another formulation.
- Follow, ShiftOut, Pass, Return, Rejoin and Stop retain their current
  canonical production behavior.
- Unknown, Hold or any otherwise unsupported normal intent fails closed.
- `MPC::get_control()` has no legacy, progress-three-state or converted
  five-state normal solve path after canonical dispatch.
- The sole legacy `solve_problem()` owner and its private solver history are
  physically removed, not hidden by a feature flag.
- Recovery, branch/shadow solvers, candidate generation, parameters and ROS
  interfaces are unchanged.
- Do not modify or stage `aichallenge/result-summary.json`.

## Definition of Done

- Failure-first source tests reject any old normal solve fallthrough.
- Static search finds no `solve_problem()` and no legacy production resolution
  string.
- Focused and full package tests pass.
- `make autoware-build` passes.
- Dynamic trial is requested because the final normal dispatch boundary changes.
