# Results: normal authority-loss snapshot

## Implemented observation boundary

- Extracted one current-world interaction snapshot builder shared by the
  existing terminal-contingency observer and the new final-authority observer.
- The builder seals the serialized predecessor, seven-state request, exact
  physical wall world, current control prefix and V2X replay world before a
  file can be submitted.
- The new observer is called after Stop-suffix, Gate-A and previous-intent
  joins, and before missing authority is converted to external Emergency Stop.
- Snapshot persistence remains on the bounded background observation worker.
  It does not solve, publish, retain or write a production candidate store.

## Static validation

- `make autoware-build`: passed, 25 packages.
- `test_single_authority_source_contract.py`: 101/101 passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 59/59 passed.
- Existing terminal-contingency recording uses the same sealed builder and
  remains observation-only.

## Pending dynamic evidence

A bounded `make dev2` run must reproduce a final Follow authority loss and
write `normal-authority-unavailable`.  That snapshot, not the Stop symptom,
will be the A/B/C/D comparison input.
