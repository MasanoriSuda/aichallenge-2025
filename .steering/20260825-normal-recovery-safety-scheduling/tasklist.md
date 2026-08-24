# Task list

- [x] Preserve the exact overrun and code-level causal chain.
- [x] Add the pure Recovery safety-eligibility contract.
- [x] Gate both preliminary wall classification and full safety evaluation.
- [x] Add unit and source-contract coverage.
- [x] Run diff/compile/build/full package tests.
- [x] Commit the implementation.
- [x] Run dev2 and compare callback/Recovery timing.
- [x] Record whether active Recovery was exercised.
- [x] Update the migration map and close the Slice.

## Definition of Done

- Clearly moving Normal cycles perform no Recovery occupancy-grid safety work.
- Stuck detector/core update still occurs every enabled cycle.
- Low-speed candidate and every active Recovery state preserve full safety
  evaluation.
- No control authority, threshold, solver or command changes.

## Closure

Committed-source run `output/20260825-024731` exercised 5,570 skipped and 643
full Recovery safety evaluations. Skip-only windows reduced Recovery evaluation
to 0.0164 ms average while preserving continuous `VehicleMoving` detector
updates. No callback exceeded 25 ms. Active Recovery and Overtake completion
were not naturally exercised and remain `NOT EXERCISED`, not assumed valid.
