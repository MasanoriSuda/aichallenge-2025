# Results

## Frozen evidence

Snapshot:

`output/20260831-063008/d1/mpcc_architecture_snapshots/000000000992-da79aec4f6fc1402-shiftout-side-negative-dynamic-obstacle-refinement-solve-rejected/snapshot.yaml`

The unchanged comparison accepted the right production arm with:

```text
candidate_source=encounter-boundary-physical-diagonal
candidate_count=3
lattice_transition=5
lattice_ahead=14
terminal_progress=9.07251
terminal_velocity=3.36381
lateral_reserve=1.50726
bundle=1
```

The left sibling failed exact dynamic proof due to a new overlap. This is
`A/B fail, C succeeds`: candidate generation defect.

## Static validation

- focused stateless maneuver tests: 27/27 passed;
- package tests: 59/59 targets, 2266 tests, zero failures;
- `make autoware-build`: 25 packages built successfully;
- JSON registry validation and `git diff --check`: passed.

## Bounded dynamic validation

Run: `output/20260831-072258`, `make dev3`, approximately two minutes.

D1 and D2 both reached `Idle -> ShiftOut -> Pass` under certified seven-state
normal authority. The finite-boundary candidate family was not selected in this
run; observed selected sources were DirectSide and LateExactDisjunction, so the
specific new candidate remains dynamically unexercised rather than dynamically
proven.

D1 later failed at a distinct Return-entry family:

```text
Pass -> FollowPrepare
build_detail=canonical current-epoch target tube unavailable
terminal-contingency-unavailable
Emergency Stop
actual footprint intersects static wall
```

This later failure does not falsify the frozen candidate-generation result and
must not be hidden by changing candidate clearances or retention rules. It is
the next root-cause Slice.
