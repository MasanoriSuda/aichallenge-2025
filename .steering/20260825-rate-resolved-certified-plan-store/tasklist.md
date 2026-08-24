# Task list

- [x] Audit the split numerical/physical mailbox boundary.
- [x] Add pure certified-plan validation and monotonic store.
- [x] Add deterministic reject and preservation tests.
- [x] Connect the store after physical acceptance in the existing worker.
- [x] Add shadow-only runtime/source-contract telemetry.
- [x] Run build and package tests.
- [x] Run a bounded dynamic shadow Gate (`output/20260825-054114`).
- [x] Record evidence and commit without generated output artifacts.

## Dynamic Gate result

- D1 retained 403 accepted certified plans; D2 retained 814.
- Certification reject, invalid replacement and stale replacement were zero.
- The final trace reported `cert_reason=none/last=accepted`.
- Shadow solver failure and control callback overrun were zero in both domains.
- Every trace remained `authority=shadow, selected=0`.
- D1 entered Overtake after the initial Track/Cruise interval, so its smaller
  count is an intent-scope observation rather than a store failure.
