# Results

## Root-cause conclusion

The tactical `ShiftOut -> Pass` transition and canonical command publication
are intentionally not atomic. The publisher retained ShiftOut correctly, but
the Pass-entry wall gate stopped querying its exact published certificate based
only on the tactical phase label. This created a false certificate gap and the
old `no valid current-side prefix` abort.

The correction queries the executed ShiftOut artifact during tactical Pass as
well. Acceptance is still controlled by the existing exact intent, target,
Mission generation, side, immutable source time, and publication-cursor join.
No time-based retention rule was added.

## Static verification

- Failure-first focused contract test: failed before implementation, passed
  after implementation.
- `test_single_authority_source_contract.py`: 82/82 passed.
- `make autoware-build`: 25 packages passed.
- Package CTest: 55/55 passed.
- `git diff --check`: passed.

No solver, clearance, speed, lease, timeout, fallback, or Recovery parameter
changed.

## Dynamic verification

- Run: `output/20260830-113908`
- Scenario: `make dev2`
- Domain 1:
  - `Idle -> ShiftOut`: observed.
  - `ShiftOut -> Pass`: decision 2067, generation 1, side +1.
  - canonical atomic admission retained ShiftOut while Pass was proposed.
  - published ShiftOut alignment remained active during tactical Pass.
  - `Pass entry physical gate held`: 0.
  - `no valid current-side prefix`: 0.
  - the alignment later failed closed first on cursor exhaustion and then on
    intent mismatch after the ledger advanced.

This verifies removal of the phase-label certificate gap.

## Next frozen failure

The same run exposed a separate downstream authority gap:

- At decision 2094 the retained ShiftOut artifact became
  `progress-lift-rejected`.
- The Pass candidate had not yet joined (`proposed_world=intent-mismatch`).
- Atomic admission consequently had `no-current-world-authority` and emitted
  emergency Stop.
- A certified Pass current-world Bundle joined only at decision 2166, about
  1.81 seconds later, after the vehicle had stopped.

The later `Pass horizon extension unavailable` Recovery is also separate. It
reported `fresh target prediction unavailable` after 14.24 m of Pass travel.
Neither remaining failure should be hidden by extending the ShiftOut artifact
lease. The next Slice must classify why a current-world Pass Bundle is not
available before the physically executed ShiftOut suffix becomes unusable.
