# Requirements

## Background

The `20260815-064538` `make dev2` run entered Pass nine times, but completed
only two Returns.  Runtime wall-preplan warnings occurred 0.25--2.25 seconds
after `ShiftOut -> Pass`, so the existing exact-boundary physical gate never
owned those conflicts.  SafeSeparation was also re-entered 104 times because
nested replan holds cleared its lifecycle state, and one DynamicMissionWait
expired to Idle before rear-clear.

## Required behavior

- Keep the Pass-entry physical warning gate eligible for a short, bounded
  lease after entering Pass.
- A warning inside that lease holds the current physically valid lateral
  prefix and requests replanning; hard wall/contact faults retain priority.
- One SafeSeparation episode keeps one cumulative time/distance lifecycle
  across same-side replans and target-bound physical-prefix holds.
- A healthy target-bound DynamicMissionWait must not expire to Idle before
  rear-clear.  It remains eligible for replan until rear-clear, a genuine hard
  fault, or the existing total Mission budget.

## Constraints

- Do not weaken wall contact, hard wall margin, emergency-front-risk, solver,
  forbidden-waypoint, target-jump or non-recoverable overlap guards.
- Do not increase the existing physical hold time/distance budget.
- Preserve target ID, side and Mission generation while a hold is valid.
- Do not change ROS topics, messages, services, launch entry points or result
  schemas.
- Preserve the user's D2 acceleration experiment and result JSON changes.

## Definition of done

- Core tests cover an early-Pass gate window and rear-clear retention at pause
  expiry.
- Controller state is not rearmed by nested SafeSeparation replan holds.
- Local/cloud YAML contain identical new lease parameters.
- Package build and tests pass.
