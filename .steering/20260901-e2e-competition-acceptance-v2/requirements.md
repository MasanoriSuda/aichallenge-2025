# Requirements

## Objective

Freeze the admitted TinyLidarNet production candidate and make an E2E race run
auditable as one immutable unit.  A run must not be called successful from bag
motion alone: Finish, lap count, penalties, stall metrics, domain identity, and
runtime controller provenance must agree.

## Frozen production contract

- checkpoint:
  `aichallenge/ml_workspace/tiny_lidar_net/checkpoints/20260901_055824/candidate.npy`
- SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- lateral controller: `tiny_lidar_net`
- runtime mode: `fixed_lidar_brake`
- no recurrent/residual/teacher production promotion in this slice
- no steering, braking, or clearance tuning in this slice

## Required behavior

1. AWSIM writes `result-summary.json` and `dN-result-details.json` below the
   same `/output/<run_id>/` that owns `awsim.log` and `dN/`.
2. A new competition analyzer combines the existing motion admission JSON,
   AWSIM result JSON, and the launch-time TinyLidarNet provenance.
3. Missing or mismatched artifacts are `incomplete`/`fail`, never `pass`.
4. Old motion-only runs remain useful evidence but cannot be silently promoted
   to competition acceptance.
5. Existing result schema, filenames, ROS interfaces, and `output/latest/`
   contracts remain unchanged.

## Definition of Done

- shell and Python syntax checks pass
- unit tests cover success, missing result, unfinished race, penalty, domain
  mismatch, provenance conflict, and checkpoint mismatch
- a fresh E2E run stores result JSON in its run directory
- a fresh run can be classified by the combined analyzer
- candidate3 hash and production runtime setting are unchanged
- generated `output/` and root-level AWSIM JSON are not committed
