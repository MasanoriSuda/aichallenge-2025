# Requirements

## Purpose

Prepare the committed ShiftOut/Pass ownership code for the next behavioral
change by separating shared Mission hard guards from Pass geometry ownership.

## Scope

- Refactor only the pure ownership policy in `v2x_overtake_core`.
- Keep the existing public `can_preserve_committed_*_behavior()` entry points.
- Expose the common guard and Pass geometry resolutions for focused tests and
  the subsequent ContactContinuation ownership change.
- Add unit tests for the extracted decisions.

## Constraints

- Do not change runtime thresholds, configuration, ROS interfaces, topics, or
  state transitions.
- Do not make confirmed overlap eligible for committed Pass ownership in this
  task.
- Preserve all existing fail-closed conditions.
- Do not modify generated output or rosbag files.

