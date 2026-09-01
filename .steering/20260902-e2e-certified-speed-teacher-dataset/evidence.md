# Evidence

## Production boundary

- Production authority was not changed.
- Frozen v11 model SHA-256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`
- Frozen base checkpoint SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- No ROS topic, service, launch-entry or submission interface changed.

## Executed teacher source

- Run: `/output/20260902-e2e-speed-committed-seed2033`
- Control mode: `speed_committed_teacher`
- Outcome: Finish 3/3, penalty 0, stall 0
- Lap times: 104.0874329 / 91.0495987 / 90.9996262 s
- Total: 286.1366577 s
- LiDAR minimum positive range: 2.58664 m
- Outcome certificate SHA-256:
  `03d53cf0e2c8105a41c7dc15cad4d9bed2a62aa3dca11ae567ed2b4b1d19a4a3`

## Immutable raw dataset

- Root: `dataset/speed_committed_seed2033_full_v1`
- Split: `val`
- Sequence:
  `eed_sync-latest_preceding-max_speed_age-0.05-max_duration-None-split-val-ff3f035a17`
- Samples: 6100 / 6100 scans
- Active-only filtering: false
- Raw label source: `lidar_speed_committed_teacher_dagger`
- Speed synchronization: latest preceding, no future samples
- Mean / p95 / max speed age:
  0.0113712835 / 0.0224438402 / 0.0402963610 s
- Metadata SHA-256:
  `7b331be24d03971fddac67c2aa3ab739e14d7d632a6151773771047d1fca5ad6`
- Scans SHA-256:
  `93760ebec96fb4c628ef60dbc1f603d202ce470fe029d3f244d9f135db51325a`
- Speeds SHA-256:
  `13d8f6b37dc84b4d2349494fce1a3095d82840eb6b1f5252d63de5ee12e32ced`
- Successor steering SHA-256:
  `674b6fe348034a972dcc80c562020621339bfe8ae7b5c923090365e2996a0187`
- Frozen-base steering SHA-256:
  `cea2c39be48ca1d8ce4188d33f18b2dba9963ab8ad57ca7cffad85bdcafb1b3a`

The earlier 1099-sample active-only extraction is retained only as an ignored
diagnostic artifact named `speed_committed_seed2033_active_diagnostic_v1`.
It is inadmissible as a temporal training source because filtering destroys
the stateful teacher's scan-by-scan transition history.

## Derived contract validation

An ephemeral recurrent derivative was built from the complete source and read
back through `RecurrentPolicySequenceDataset`:

- samples: 6100
- recurrent label source:
  `lidar_speed_committed_teacher_recurrent_direct`
- maximum inherited speed age: 0.040296361 s
- inherited certificate SHA-256:
  `03d53cf0e2c8105a41c7dc15cad4d9bed2a62aa3dca11ae567ed2b4b1d19a4a3`

The builder rejects an active-only speed-committed source, mismatched teacher
mode, future speed, stale speed, source/certificate mismatch and label-source
reuse.

## Validation

- `python3 -m py_compile`: pass
- focused container tests: 72 passed
- actual 6100-sample raw-to-recurrent contract check: pass
- full ML workspace tests: 194 passed

## Decision

This Slice certifies data provenance only. Seed 2033 remains held-out
validation evidence. No model is trained or promoted until an independent
successful train run exists and static-versus-temporal representability is
measured on run-disjoint data.
