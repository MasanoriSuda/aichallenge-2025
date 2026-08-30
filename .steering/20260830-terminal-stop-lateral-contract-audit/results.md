# Results

## Frozen source

`output/20260830-200852/d1/mpcc_architecture_snapshots/000000004017-ee88c9e56718aeeb-shiftout-side-positive-physical-proof-terminal-contingency-unavailable/snapshot.yaml`

## Comparison

| Arm | Result |
|---|---|
| racing-line fixed Stop | wall contact near waypoint 318 |
| Mission-declared fixed lateral Stop | same wall contact |
| 128 fixed lateral targets over physical range | no certified target |
| seven-state Stop from current state | accepted, but not causal enough for promotion |
| seven-state Stop after exact 25 ms publisher prefix | accepted |

The causal arm produced:

- terminal velocity: `-5.0012e-14 m/s` (numerical zero);
- terminal progress: `13.4194 m`;
- minimum exact lateral reserve: `0.293009 m`;
- exact wall proof: accepted;
- exact current-world dynamic proof: accepted;
- solve time in offline comparison: `96.1953 ms`.

## Classification

This snapshot is not physically infeasible.  The normal ShiftOut trajectory
and a recursively viable Stop both exist under the same physical limits.

The root defect is the terminal candidate family: immediate maximum braking
combined with a single fixed lateral feedback target cannot represent the
coupled steering/braking path required by the wall geometry.  Merely wiring
`ContingencyStopIntent::hold_lateral_m` is refuted.

The next production Slice must replace the canonical fixed-policy terminal
Stop candidate with a certified seven-state Stop artifact.  It must retain one
Stop owner and identical exact proof/publisher semantics; the old candidate
must not remain as a permanent fallback.

## Static verification

- `make autoware-build`: 25 packages completed.
- `test_mpcc_rate_resolved_physical_adapter`: 23/23 passed.
- `test_race_mpcc_foundation`: 35/35 passed.
- `test_mpcc_architecture_comparison`: 20/20 passed.
- `test_single_authority_source_contract.py`: 91/91 passed through colcon.
- full `multi_purpose_mpc_ros`: 2263 tests, 0 errors, 0 failures.
