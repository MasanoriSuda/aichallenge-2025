# Requirements

## Purpose

Improve the overtake path decision after the 2026-08-15 run showed that the
configured 30 m Frenet-DP lookahead produced about 52 m of actual path and that
confirmed `target clear ahead` events could fall into `DynamicMissionWait`.

## Scope

- Count overtake planning horizons from actual waypoint arc length.
- Keep 30 m available for corridor feasibility, but limit curve-tactic
  influence to the nearest 20 m.
- Let a target confirmed clear ahead enter a validated Return at the same 2 m
  threshold used by SafeSeparation.
- Preserve current-body, wall/Return-corridor and hard-fault guards.

## Constraints

- Do not change ROS 2 topics, message types, launch entry points or evaluator
  interfaces.
- Do not relax current physical overlap, wall or hard-fault checks.
- Do not modify generated output or rosbag artifacts.

## Definition of done

- Core unit tests cover tactical-horizon isolation and confirmed-clear Return.
- The package builds and its tests pass.
- A subsequent `make dev2` log reports a DP path near 30 m rather than 51-52 m.
