# Evidence

## Admitted teacher sequence

- Source run: `output/20260901-054831`
- Source controller: `gap_teacher`
- Source motion: 1020.22 m in 298.01 s
- Source post-start stall: 0 s
- Extracted sequence:
  `output-20260901-054831-d1-rosbag2_autoware-217ab81551`
- Label source: `lidar_gap_teacher`
- Split: train
- Samples: 5,949 (0 synchronization rejects)

The aggregate train split contains 13,592 samples from three independently
identified sequences: the original admitted teacher run, round-one DAgger
corrections, and this successful seed-2027 teacher rollout. The original 6,130
sample run-level validation split is unchanged.

## Candidate3

- Training run: `checkpoints/20260901_055824`
- Pretrained candidate2 SHA-256:
  `b6569bde0bbaa72c43a2534358073c72e6572680b6907ecd9a881e8bf5aef5d8`
- Candidate3 SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`

Independent validation against candidate2:

| Metric | candidate2 | candidate3 | Change |
|---|---:|---:|---:|
| RMSE | 0.042021 rad | 0.041860 rad | 0.38% better |
| MAE | 0.015265 rad | 0.015784 rad | 3.40% worse |
| corrective RMSE | 0.102895 rad | 0.101688 rad | 1.17% better |
| corrective MAE | 0.060505 rad | 0.058636 rad | 3.09% better |

## Closed-loop admission

| Gate | Run | Distance | Duration | Post-start stall | Result |
|---|---|---:|---:|---:|---|
| Single vehicle | `20260901-055914` | 1007.41 m | 295.22 s | 0 s | pass |
| NPC seed 2027 | `20260901-060540` | 1019.38 m | 308.19 s | 0 s | pass |
| NPC seed 2028 (unseen) | `20260901-061228` | 1015.22 m | 328.63 s | 0 s | pass |

In both NPC gates the longitudinal safety authority activated during close
approach and subsequently returned to clear state. Unlike candidate2, candidate3
continued forward after intervention rather than remaining stopped.

## Promotion decision

Candidate3 passes the required normal, reproduced and unseen closed-loop gates.
It replaces the previous production checkpoint whose SHA-256 was
`8ebd93687b05b17cc20741fa34c7e78bec05d380993ddd6ff75ddb6e49d3af08`.
The promoted artifact must hash to the candidate3 identity above.

## Shipped artifact verification

- `make autoware-build`: 25 packages passed.
- Package tests: 2,291 tests, 0 errors, 0 failures, 0 skipped.
- Source checkpoint and container install-space checkpoint both hash to
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`.
- `colcon test-result` reported one stale unrelated missing
  `joycon_contract_guard/package.xml` result path before the successful aggregate
  test summary; no selected package test failed.
