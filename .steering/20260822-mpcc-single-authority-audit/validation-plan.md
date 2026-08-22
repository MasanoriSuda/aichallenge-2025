# Validation and replay plan

## Principles

1. Freeze code and config for all repeats of one experiment.
2. Compare against the immediately previous accepted slice, with `dc51093` as the Phase 0 baseline.
3. Do not mix structural migration and weight/clearance tuning.
4. Report simulation evidence as simulation evidence.
5. A fast lap does not compensate for an authority violation, stale result, collision, or abnormal
   tail lap.

## Deterministic replay scenarios

| ID | Scenario | Primary invariant |
|---|---|---|
| R01 | Clean Track over straight, curve and circular seam | I-04, I-06, I-07 |
| R02 | Cruise with no V2X vehicles | I-04, I-14 |
| R03 | Moving lead vehicle at stable speed; gap closes then stabilizes | I-03, I-04 |
| R04 | Nearly stopped lead vehicle with left side open | I-01, I-08 |
| R05 | Nearly stopped lead vehicle with right side open | I-01, I-08 |
| R06 | Both pass sides available; asynchronous left/right results complete in opposite orders | I-11, I-12 |
| R07 | Selected side becomes blocked before no-return; opposite side remains connected | I-02, I-13 |
| R08 | Dynamic Escape fresh candidate has a one-cycle gap | I-02, I-13 |
| R09 | MPCC build rejection, max-iteration failure and non-finite result injected separately | I-01, I-05, I-13 |
| R10 | Physical wall certificate rejects the solved trajectory | I-01, I-02 |
| R11 | Target observation generation changes while async solve is running | I-08, I-11 |
| R12 | Hold/Stop at zero or very low speed without virtual-progress reward leakage | I-04, I-15 |
| R13 | Pass reaches body-clear and rear-clear, then Return completes | I-04, I-12 |
| R14 | Contact/wall stuck episode transfers to Recovery and returns | I-09, I-10 |

Replays should be extracted from real run inputs where possible. The current high-value seed is the
encounter around decisions 1386-1393 and 3504-3514 in
`output/20260822-105057/d1/autoware.log`.

## Required telemetry

Each published command transition must identify:

```text
decision_id
intent / intent_generation
target_id / obstacle_generation
problem_fingerprint
formulation_schema
solution_id
certificate_id
lateral_solution_id
longitudinal_solution_id
solution_age / remaining_stages
solver_status / iterations / solve_time
minimum_wall_reserve / minimum_vehicle_reserve
normal_authority | emergency_override | recovery_override
override_reason
```

Normal heartbeat logs may be aggregated, but authority, formulation, certificate, and override changes
must be emitted immediately.

## Slice KPIs

### Correctness

- selected but unsolved/uncertified candidates: `0`;
- solution/certificate/final fingerprint mismatch: `0`;
- lateral/longitudinal solution mismatch: `0`;
- stale async result adoption: `0`;
- no-return opposite-side switch: `0`;
- schema-only candidate selected: `0`.

### Authority

- normal formulation switches after an intent is promoted: `0`;
- normal command authorities after Slice 6: `1`;
- allowed external overrides after Slice 6: Emergency and Recovery only;
- legacy/direct normal command publication after its deletion slice: `0`.

### Safety

- non-finite command: `0`;
- wall/contact events introduced by the slice: `0` relative to the accepted baseline;
- invalid wall/opponent certificate publication: `0`;
- Recovery entered without upstream failure identity: `0`.

### Real time

The control period is 25 ms at 40 Hz. Record mean, p95, p99, maximum and consecutive overruns for:

- problem construction;
- branch worker;
- canonical solve;
- physical certification;
- full control callback.

Initial acceptance requires no consecutive callback overrun that causes a stale command. Exact p99
budget allocation must be fixed in the implementation slice after measuring the Phase 1 contracts;
Phase 0 does not invent a tuning threshold.

### Reproducibility

For dynamic acceptance, run at minimum:

- single vehicle clean Track;
- two vehicles with the same initial placement and config;
- three vehicles where non-target intrusion is possible;
- six laps when the scenario remains controllable;
- multiple repeats/seeds where AWSIM supports them.

Report all repeats, not only the best run.

## Phase 0 baseline observations

The following are evidence for migration tests, not acceptance metrics:

- `20260822-105057` D1 contains seven logged extended-MPCC-unavailable warnings and twenty log lines
  containing `solver-fallback` (change/diagnostic lines, not necessarily twenty distinct control
  cycles).
- The same log contains Dynamic Escape execution, wall requalification holds, retained execution,
  extended and legacy solves, and an explicit multiple-lateral-authority conflict.
- D2 does not show the same activity, which makes D1 the primary reproduction source rather than
  evidence of a global startup failure.

## Acceptance report template

Every slice report must include:

1. repaired invariant;
2. earliest pre-fix violation;
3. deterministic failing replay before the fix;
4. passing replay after the fix;
5. production branches/configuration removed;
6. remaining competing authorities;
7. safety and timing evidence;
8. all dynamic run results;
9. rollback commit;
10. next slice gate.
