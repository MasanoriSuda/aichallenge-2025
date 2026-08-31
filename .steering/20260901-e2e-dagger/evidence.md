# Evidence

## Relabeling

- Source failure: `output/20260901-044333`
- Student checkpoint SHA-256:
  `84da0c3ccec5d80d1bc1177a9eddcb93a928ed30fdd50b43cc2c6c09f797e195`
- Raw scans: 8,498
- Confirmed contact breach index: 6,033
- Exclusive cutoff after the one-second margin: 6,014
- Accepted active teacher corrections: 1,632
- Student control messages used as labels: 0

## Candidate

- Training run: `checkpoints/20260901_050440`
- Candidate SHA-256:
  `b6569bde0bbaa72c43a2534358073c72e6572680b6907ecd9a881e8bf5aef5d8`
- Independent normal validation versus candidate 1:
  - RMSE: `0.04191 -> 0.04202 rad` (0.25% regression)
  - MAE: `0.01385 -> 0.01527 rad` (10.2% regression)
  - corrective-subset RMSE: `0.11385 -> 0.11140 rad`
  - corrective-subset MAE: `0.06563 -> 0.06271 rad`

## Closed loop

| Gate | Result | Distance | Duration | Positive-accel stall |
|---|---:|---:|---:|---:|
| single vehicle `20260901-050714` | pass | 1,009.83 m | 297.48 s | 0.00 s |
| NPC seed 2026 `20260901-051441` | pass | 1,016.93 m | 308.80 s | 0.00 s |
| NPC seed 2027 `20260901-052104` | fail | 897.12 m | 425.15 s | 79.56 s |

The seed 2027 failure starts with the fixed `+0.6 m/s2` acceleration command at
approximately 0.48 m frontal clearance. The teacher's braking output is present
in generated data, but steering-only training and fixed-acceleration runtime do
not consume it. This is an interface mismatch between the corrective teacher and
the deployed student, not evidence for another steering threshold change.

## Decision

The deterministic relabeling infrastructure is accepted. Candidate promotion is
rejected. Production weights remain unchanged. The next slice must close the
longitudinal safety contract while retaining ML ownership of lateral control.
