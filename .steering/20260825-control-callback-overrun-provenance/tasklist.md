# Task list

- [x] Preserve both overrun examples.
- [x] Separate plausible timing owners from symptoms.
- [x] Add stack-local timing observation and exact overrun trace.
- [x] Add observation-only source contract.
- [x] Run build and full package tests.
- [x] Commit the diagnostic Slice.
- [x] Run dev2 and exercise at least one overrun or record `NOT EXERCISED`.
- [x] Identify the dominant region before any optimization.

## Definition of Done

- An overrun line identifies decision, total/budget, all five regions,
  unattributed time and completion checkpoint.
- Existing aggregate telemetry remains available.
- No control result, authority, solver or cadence changes.

## Closure

The second dev2 run exercised one exact overrun. `MPC::get_control()` owned
20.786 ms and Stuck Recovery evaluation owned another 5.046 ms; post-MPCC
arbitration and publication were negligible. The Slice is closed as a
diagnostic result. Any scheduling repair belongs to a new causal Slice.
