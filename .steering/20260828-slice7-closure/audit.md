# Audit

## Accepted structural baseline

Commit: `b273d56d`

Run: `output/20260828-044759`

- `N=20` in local and cloud production configuration.
- Production solve submission at the 40 Hz control cadence.
- Four `Idle -> ShiftOut` entries.
- Two `ShiftOut -> Pass` transitions.
- One `Pass -> Return -> Idle` completion.
- Overtake Recovery: 0.
- Actual-footprint wall-margin violation: 0.
- Callback overruns: 102/5713 cycles (1.785%).
- Maximum callback/MPCC cycle: 56.647/56.310 ms.

The run proves one complete dynamic Overtake lifecycle.  It does not prove
production readiness across starts, seeds, and all encounter geometries.

## Rejected candidates

| Candidate | Positive result | Decisive regression | Decision |
|---|---|---|---|
| `N=16` | Lower maximum MPCC cycle | No complete Return/Idle; DynamicWait Recovery | rejected and reverted |
| `N=18` | First run completed two episodes with lower tail | Independent run added wall Recovery/violation | rejected and reverted |
| `N=20`, solve at 20 Hz | Scheduler reduced submissions | No Pass; Rejoin authority hole repeated 40 times | rejected and removed |

## Production-state verification

- Both production YAML files contain `N: 20`.
- No 20 Hz production cadence config, telemetry, or source branch remains.
- The candidate horizon and cadence changes produced no retained source diff.
- User/generated result and symlink-manifest files were not staged or edited.

## Remaining non-tuning work

- Callback and worker p95/p99/max tail reduction without shortening the proof.
- Maximum-iteration root-cause classification on replay-ready exact snapshots.
- Elimination of canonical-normal Emergency intervals through certificate and
  lifecycle continuity, not through a lease or tolerance relaxation.
- Repeated dynamic acceptance across starts, sides, and encounter geometries.

These items reopen architecture/integration investigation.  They do not
justify another parameter family inside this Slice 7 campaign.
