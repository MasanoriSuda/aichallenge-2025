# Validation

## Static gates

- `git diff --check`: passed.
- Source-contract pytest with third-party plugin autoload disabled: 19 passed.
- `make autoware-build`: 25 packages built successfully. The only stderr was
  the existing setuptools deprecation warning.
- `colcon test --packages-select multi_purpose_mpc_ros`: 40/40 CTest entries
  passed. `colcon test-result` reported 1,789 tests, zero errors and zero
  failures. It also reported the pre-existing missing
  `build/joycon_contract_guard/package.xml` metadata while scanning unrelated
  build results.

## Dynamic Gate

- Command: clean `make dev2`, no manual pose or control publication.
- Output: `output/20260824-200419`.
- Configuration changes: none.

At `1787569501.336`, a cached-fresh progressive entry at 10.02 m reported:

```text
physical=certified, speed_proof=certified-execution
```

The same transaction then produced:

```text
OvertakeLine: Idle -> ShiftOut
canonical_intent=shiftout
formulation=velocity-progress-5state
canonical_source=retained-certified
contract_join=1
```

This closes the reproduced false `minimum-speed-insufficient` admission from
`output/20260824-194340`. The 8 m unproven-completion reserve and every wall,
target, body, freshness and current-world Gate remained unchanged.

## Separate downstream result

This run does not complete the Slice 5 intent matrix. Approximately 4.4 s
after entry, the vehicle stopped during ShiftOut and Stuck Recovery observed
`wall=rear, wall_distance=0.018 m`. Recovery authority then superseded the
still-valid canonical ShiftOut command and the episode ended with
`external recovery completed`.

Therefore:

- certified fresh-entry speed proof: **accepted**;
- ShiftOut canonical authority: **exercised and joined**;
- Pass: **not exercised**;
- Return: **not exercised**;
- collision-free Overtake quality: **not accepted**.

The wall-contact/stop chain is a separate earliest failure and must be audited
in another bounded Slice. It must not be hidden by changing the entry speed
proof, the 8 m completion Gate, wall margin, solver settings or Recovery policy.
