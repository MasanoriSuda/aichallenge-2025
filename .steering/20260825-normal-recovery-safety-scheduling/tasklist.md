# Task list

- [x] Preserve the exact overrun and code-level causal chain.
- [x] Add the pure Recovery safety-eligibility contract.
- [x] Gate both preliminary wall classification and full safety evaluation.
- [x] Add unit and source-contract coverage.
- [x] Run diff/compile/build/full package tests.
- [ ] Commit the implementation.
- [ ] Run dev2 and compare callback/Recovery timing.
- [ ] Record whether active Recovery was exercised.
- [ ] Update the migration map and close the Slice.

## Definition of Done

- Clearly moving Normal cycles perform no Recovery occupancy-grid safety work.
- Stuck detector/core update still occurs every enabled cycle.
- Low-speed candidate and every active Recovery state preserve full safety
  evaluation.
- No control authority, threshold, solver or command changes.
