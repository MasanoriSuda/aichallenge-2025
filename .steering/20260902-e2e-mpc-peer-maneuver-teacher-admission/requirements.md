# Requirements

## Objective

Evaluate the current MPC/MPCC controller as a complete-maneuver teacher in the
same deterministic three-vehicle peer world that exposed the E2E teacher's
missing escape certificate.  Admission precedes extraction or learning.

## Hypothesis

The E2E heuristic selects an instantaneous polar gap and cannot prove a
complete escape trajectory.  The current MPC/MPCC stack already represents a
time-indexed trajectory, vehicle footprint, wall/opponent constraints and a
terminal successor.  A successful ego run can therefore provide LiDAR-to-
steering imitation labels for complete maneuvers without adding that planner
to E2E runtime.

## Frozen experiment

- World: `e2e-peer`, three controllable vehicles.
- All domains: current `mpc` controller with no TinyLidar experiment overrides.
- Primary teacher candidate: domain 3.
- Scenario, controller configuration and acceleration remain unchanged.

## Constraints

- A valid label source must Finish, have zero penalty and zero sustained stall.
- A run that merely moves or completes one overtake is insufficient.
- Do not extract failed MPC commands as labels.
- Do not mix this run into existing datasets before run-level admission and a
  run-disjoint split plan.
- Do not alter MPC/MPCC parameters to make the teacher pass in this Slice.
- E2E production authority and artifacts remain unchanged.

## Definition of Done

- Run the frozen three-peer MPC world once.
- Analyze motion, result/penalty and runtime provenance for domain 3.
- If admitted, create an immutable extraction plan; if rejected, record the
  failure and stop before dataset/model changes.
