# Task list

- [x] Identify the runtime/admission shift-distance mismatch in the latest log.
- [x] Define the nominal-plan versus runtime-window contract.
- [x] Make admission shift sizing aware of the candidate command speed.
- [x] Recompute the runtime shift within the remaining scheduled window.
- [x] Add focused regression coverage and failure diagnostics.
- [x] Run the package build and focused tests.
- [x] Record verification results and dynamic acceptance criteria.

## Dynamic check remaining for the operator

- [ ] Run `make dev2` through at least one scheduled outside-role reversal.
- [ ] Confirm `scheduled outer transition accepted` is emitted.
- [ ] Confirm `shift` can exceed `nominal_shift` while remaining below
  `available_shift`.
- [ ] Confirm the side handoff reaches rear-clear and `Pass -> Return -> Idle`
  without contact, wall Recovery or prolonged hard braking.

