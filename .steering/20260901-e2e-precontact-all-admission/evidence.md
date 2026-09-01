# Evidence

## Frozen setup

- world: `e2e-final` (`4 vehicles`, `6 laps`, synchronized start)
- checkpoint SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- d1--d4 control mode: `precontact_teacher`
- production defaults: unchanged

All four launch logs reported the requested diagnostic mode before the race was
started.

## Runs

### Initial duration audit

`output/20260901-094728` was stopped manually after about 542 seconds of bag
time.  Every domain passed both motion gates, but this run did not supply a
terminal event and therefore was not used to admit extraction.

| Domain | Distance [m] | Duration [s] | Longest low speed [s] | Positive-accel stall [s] |
|---|---:|---:|---:|---:|
| d1 | 1744.22 | 541.86 | 0.0 | 0.0 |
| d2 | 1784.77 | 542.62 | 0.0 | 0.0 |
| d3 | 1760.23 | 542.10 | 0.0 | 0.0 |
| d4 | 1799.80 | 541.66 | 0.0 | 0.0 |

### Terminal rerun

`output/20260901-100204` ran without intervention until `/awsim/state` became
`Finish`.  The autostart orchestrators then stopped each recorder and generated
motion analytics.  No `result-summary.json` is expected from this non-evaluation
practice harness; terminal admission comes from the AWSIM state/orchestrator
stop path rather than an evaluation result schema.

All finalized bags pass the same `analyze_e2e_run.py` gates:

| Domain | Distance [m] | Duration [s] | Mean speed [m/s] | Low speed [s] | Positive-accel stall [s] | Result |
|---|---:|---:|---:|---:|---:|---|
| d1 | 1715.10 | 551.07 | 3.107 | 0.0 | 0.0 | pass |
| d2 | 1757.16 | 550.61 | 3.187 | 0.0 | 0.0 | pass |
| d3 | 1737.52 | 550.34 | 3.154 | 0.0 | 0.0 | pass |
| d4 | 1770.28 | 550.62 | 3.211 | 0.0 | 0.0 | pass |

The bags contain roughly 4,470 LiDAR samples per domain.  Runtime logs continued
to roughly 6,730 controller scans while finalization completed; no controller
stale interval was reported.

## Admission decision

The symmetric pre-contact teacher is admitted as an **offline corrective-label
source** for these four finalized bags.  This does not promote the teacher to
production authority and does not change the production checkpoint.

Extraction must preserve explicit provenance:

- label source: `lidar_precontact_teacher_dagger`
- teacher: `LidarPrecontactTeacher`
- control mode: `precontact_teacher`
- source run/domain identity and checkpoint hash

The historical `lidar_gap_teacher_dagger` identity may not be reused because it
would falsely identify a materially different detection and projection policy.

## Remaining limitation

The practice harness emits no evaluation result JSON.  Collision/penalty scoring
must therefore be checked again in a later evaluation-compatible gate before a
new student checkpoint is promoted.  Corrective extraction remains guarded by
the pre-contact cutoff and by independent closed-loop student validation.
