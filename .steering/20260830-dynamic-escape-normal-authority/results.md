# Results: DynamicEscape normal-authority integration

## Root-cause conclusion

The baseline did not lose authority because the current world had no lateral
avoidance solution. A compatibility producer promoted pre-Mission
`DynamicEscape` to canonical `ShiftOut`, while the later bounded normal
current-world population continued to solve the same encounter as `Cruise`.
The intent mismatch discarded the proved normal artifacts before Gate A could
run, because no real OvertakeLine Mission existed yet.

The fix deletes that old producer. Pre-Mission avoidance remains a tactical
`DynamicEscape` action but is solved and published as normal Track/Cruise
dynamic-obstacle avoidance. Only an admitted OvertakeLine Mission may create
canonical ShiftOut/Pass/Return identity.

## Static verification

- source-contract tests: 76 passed;
- package CTest: 54 targets, 2157 tests, zero failures;
- `make autoware-build`: 25 packages passed;
- `git diff --check`: passed.

An initial host pytest invocation auto-loaded ROS launch plugins and collected
`localization_scope`; it failed before the focused contract ran. Repeating the
focused test with third-party plugin autoload disabled passed 76/76. The Docker
package tests, which are the repository's supported ROS environment, also
passed completely.

## Dynamic comparison

Baseline: `output/20260830-001650`, commit `e3ac36e4`.

- the false signature
  `canonical_intent=shiftout/resolved-action` with
  `lateral_owner=dynamic-obstacle-escape` occurred 30 times;
- four real `Idle -> ShiftOut` transitions and one `ShiftOut -> Pass` occurred;
- 52 Emergency-override decisions were recorded.

Candidate: `output/20260830-004030`.

- the false pre-Mission ShiftOut signature occurred zero times;
- six real `Idle -> ShiftOut` transitions and one `ShiftOut -> Pass` occurred,
  so deleting the compatibility producer did not remove real Overtake entry;
- 19 Emergency-override decisions were recorded.

The two runs have different lengths and encounters, so the Emergency totals
are not a performance-rate claim. The acceptance evidence for this Slice is
the structural deletion and the disappearance of its unique false-authority
signature. The candidate run did not happen to activate pre-Mission
`Action::DynamicEscape`; its Track/Cruise mapping is therefore dynamically
unexercised and covered by the focused unit and source-contract tests.

## Separate residual failure families

The candidate still exposed failures which are not caused by the deleted
producer and must not be hidden by this Slice:

- decision 1382: ShiftOut to Cruise handoff with
  `terminal-contingency-unavailable`;
- decision 1473: DynamicWait with no executable canonical lateral authority;
- decision 1513: ShiftOut to Cruise handoff with `progress-lift-rejected`;
- decision 2060: ShiftOut to Pass handoff with
  `terminal-contingency-unavailable`;
- Pass-time SafetyBrake and one FollowPrepare-to-Recovery transition.

These snapshots define the next root-cause audit. No lease, grace, retry,
fallback, solver tolerance or clearance change was added here.
