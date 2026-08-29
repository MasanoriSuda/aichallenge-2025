# Results

## Root-cause conclusion

The blocked `ShiftOut -> Pass` transition was not a physical-infeasibility
result. The canonical seven-state artifact was already being published under
accepted current-world proof while an approximate legacy projection acted as a
second certificate owner and rejected it with `requires wall clamp`.

The fix assigns Pass-entry certification to the matching, actually-published
canonical artifact. The projection remains available only on the migration path
where no canonical published identity exists. A missing canonical cursor fails
closed and cannot fall through to that weaker source.

## Static verification

- `test_single_authority_source_contract.py`: 81 passed.
- `make autoware-build`: 25 packages passed.
- Package CTest: 54/54 passed.
- `git diff --check`: passed.

No solver option, clearance, speed, lease, timeout, fallback, or recovery
parameter changed.

## Dynamic verification

- Run: `output/20260830-031429`
- Scenario: `make dev2`
- Domain 1 observations:
  - `Idle -> ShiftOut`: 2
  - `ShiftOut -> Pass`: 1
  - `Pass -> Return`: 1
  - Pass-entry physical gate held: 0
  - Pass-entry physical gate expired: 0
  - canonical Pass command observed: yes
  - canonical Return command observed: yes

At decision 1662, the tactical state changed from ShiftOut to Pass while the
Pass artifact was not yet available. Atomic admission retained the accepted
ShiftOut artifact instead of stopping:

```text
previous=shiftout proposed=pass effective=shiftout
resolution=previous-retained previous_world=accepted
```

At decision 1668, the new current-world Pass Bundle joined and owned production:

```text
solver=canonical-rate-resolved-pass-current-world-bundle
intent=pass authority=certified-normal-solution
```

The same sequence repeated at `Pass -> Return`: published Pass was retained
until Return authority joined. This verifies that removing the duplicate
transition certificate did not create an authority gap.

## Separate remaining failures

The run also exposed failures outside this Slice:

- Episode 1 entered Recovery due `actual footprint wall margin violated`.
- Episode 2 completed Pass and entered Return, but later external recovery and
  prolonged stuck recovery occurred.

These are not a regression of Pass-entry ownership: the target transition now
works and the hard wall guard still fired. They should be frozen as the next
failure snapshots and classified independently instead of adding a Pass-entry
exception.

## Next evidence target

Audit the first exact current-world artifact that is published before the
Episode 1 wall-margin violation and the Return artifact/Stop transition before
external recovery. Determine whether each is candidate-generation,
certificate/model, scheduling/lifecycle, or genuine physical infeasibility.
