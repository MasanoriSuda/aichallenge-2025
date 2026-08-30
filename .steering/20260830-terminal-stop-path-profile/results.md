# Results

## Frozen replay

Source:

`output/20260830-200852/d1/mpcc_architecture_snapshots/000000004017-ee88c9e56718aeeb-shiftout-side-positive-physical-proof-terminal-contingency-unavailable/snapshot.yaml`

The audit used the unchanged normal solve, nonlinear physical adapter, exact
wall proof and current-world dynamic proof.  It added no authority path.

| Arm | Result | Evidence |
|---|---|---|
| fixed racing-line Stop | rejected | hard wall contact at exact sample 69 |
| declared fixed Stop | rejected | same hard wall contact at sample 69 |
| 128 fixed targets | rejected | no target accepted |
| normal solved-path profile Stop | rejected | same hard wall contact at sample 69 |
| causal seven-state Stop | accepted | terminal velocity approximately zero, 0.293009 m exact lateral reserve |

The normal-path profile did not change the physical failure: the persistent
arm contacted waypoint 318 at progress 314.328 m, and the production-left arm
contacted the same waypoint at progress 314.311 m.  The seven-state Stop
stopped at local progress 13.4194 m and passed exact wall and dynamic proofs.

## Follow-up correction

The later control audit found that this causal seven-state arm accelerated at
approximately `+1.33 m/s^2` for its first five stages.  It therefore did not
share the production Stop's maximum-braking longitudinal contract, and the
`13.4194 m` result must not be used as the final Stop-feasibility comparison.

The corrected maximum-braking seven-state audit is recorded in
`.steering/20260830-seven-state-stop-control-audit/results.md`.  It also passes
the unchanged exact proofs, stopping at local progress `5.55934 m` with
`0.403418 m` minimum lateral reserve and no positive acceleration.  The
path-profile rejection in this Slice is unchanged.

## Root-cause conclusion

The missing representation is not merely a varying lateral reference.  The
single path-feedback/max-braking Stop law cannot express the feasible
steering-rate sequence found by the seven-state solve.  Changing a fixed
target or replaying the normal path therefore cannot repair the candidate
family.

This refutes promotion of the path-profile candidate.  Production remains
unchanged.  The next design must make the already-proven causal seven-state
Stop an immutable asynchronous artifact without adding another publisher or
letting solver failure create a second fallback chain.

## Verification

- focused build: passed
- physical-adapter tests: 26/26 passed
- architecture-comparison tests: 20/20 passed
- full package CTest: 59/59 targets passed
- frozen decision 4017 replay: completed
