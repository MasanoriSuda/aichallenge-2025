# Requirements

## Purpose

Physically remove the now-unreachable five-state Track/Cruise normal-owner
implementation after commit `9a4da6a` promoted the certified six-state owner.
This is the first bounded Slice 6 deletion step; it is not a behavior or
parameter change.

## Root cause

Track/Cruise no longer calls the five-state canonical evaluator, but its plan
store, solver context, retained revalidation, telemetry and Track/Cruise/Rejoin
mode switch remain compiled into `mpc_controller_cpp`. Their presence obscures
the real authority graph and permits a future patch to reconnect the retired
owner accidentally.

## Required invariants

- Track/Cruise normal authority remains only
  `VelocitySteeringProgress6State` or explicit Emergency.
- Rejoin keeps its existing five-state canonical production behavior.
- The Rejoin evaluator has no Track/Cruise mode, store, solver or retained
  branch.
- No Track/Cruise five-state plan store, warm identity, solver context or
  telemetry remains in the controller.
- The rate-resolved Track/Cruise worker, certified store and production adapter
  remain unchanged.
- No parameter, timeout, fallback flag, ROS interface or Recovery policy
  changes.
- Do not modify or stage `aichallenge/result-summary.json`.

## Definition of Done

- Source-contract tests fail if a Track/Cruise five-state owner or mode returns.
- Search finds no retired Track/Cruise plan store, retained evaluator, solver
  context or five-state telemetry.
- Rejoin tests, package tests and `make autoware-build` pass.
- A short dynamic check is requested only if static removal changes a runtime
  owner boundary; otherwise the already accepted production owner remains the
  dynamic baseline.
