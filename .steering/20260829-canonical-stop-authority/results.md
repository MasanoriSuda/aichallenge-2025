# Results

## Static verification

- Focused source-contract test first failed because emergency authority still
  inherited the requested normal intent.
- Focused regression tests: 3 passed.
- `make autoware-build`: 25 packages passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 54/54 test targets
  passed; the authority source-contract file contains 75 passing cases.
- `git diff --check`: passed.

## Dynamic verification

Run: `output/20260829-162420` (`make dev2`).

At D1 decision 1353, a ShiftOut normal artifact became unavailable. The new
contract was observed exactly as designed:

- requested intent: `shiftout`;
- published emergency authority: `stop`;
- lateral action: `track-reference-path`;
- final execution contract: `intent=stop`;
- physical steering moved from approximately 0.09 rad toward the path target
  on subsequent cycles rather than holding the old ShiftOut steering;
- the prior `actual footprint wall margin violated` chain did not recur in
  this run.

The existing atomic bridge then reported `previous=stop`, retaining Stop until
a certified normal artifact became available. This confirms one Stop owner is
now used instead of a hidden non-Stop constant-steering emergency variant.

## Newly exposed failure, not patched in this Slice

The run still lost normal ShiftOut authority for extended intervals. Episode 2
shows `progress-lift-rejected` from decision 1847 until a retained prefix was
accepted at decision 1880. Correct Stop behavior therefore appeared as a
large speed loss and eventually interacted with stuck recovery.

This is separate from the fixed wall-collision propagation:

1. the current-world/Gate-A ShiftOut artifact is admitted;
2. progress lifting rejects the artifact after publication;
3. external Stop correctly owns the wire;
4. normal authority does not rejoin promptly.

The next Slice must audit the absolute/wrapped progress mapping and artifact
clock/provenance at that first rejection. It must not weaken retained proof,
add a grace period, or undo Stop authority.

## Acceptance verdict

Accepted for the bounded root cause: emergency authority identity and lateral
action are coherent, and the observed wall-seeking stop behavior is removed.
Race-quality acceptance remains open because the upstream
`progress-lift-rejected` loss of normal authority is now the limiting defect.
