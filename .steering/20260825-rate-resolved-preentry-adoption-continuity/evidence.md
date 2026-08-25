# Evidence

## Baseline observation

`output/20260825-200059` exposed three current-world rejection names but did
not retain the numeric state which caused them:

- `steering-unreachable`
- `velocity-unreachable`
- `progress-lift-rejected`

The names alone could not distinguish a coordinate/time-contract defect from
a legitimate stale-plan rejection.

## Instrumented run

Run: `output/20260825-202428`, domain 1.

The live race produced multiple fully solver/wall/target-certified ShiftOut
plans. Their adoption shadow failed after the async result had aged 0.20 to
0.47 seconds:

| sequence | age [s] | progress delta/tolerance [m] | steering delta/limit [rad] | speed expected/reachable max [m/s] | result |
|---:|---:|---:|---:|---:|---|
| 1569 | 0.320 | -0.604 / 1.500 | 0.1051 / 0.0375 | not reached | steering-unreachable |
| 1608 | 0.380 | -1.017 / 1.500 | 0.0691 / 0.2223 | 5.019 / 4.919 | velocity-unreachable |
| 1852 | 0.400 | 0.554 / 1.500 | -0.0326 / 0.0379 | 3.943 / 3.696 | velocity-unreachable |
| 2302 | 0.395 | -0.473 / 1.500 | -0.0341 / 0.0354 | 4.435 / 3.793 | velocity-unreachable |
| 2503 | 0.470 | -0.507 / 1.500 | 0.1121 / 0.0375 | not reached | steering-unreachable |
| 2555 | 0.200 | -0.053 / 1.500 | -0.0777 / 0.0363 | not reached | steering-unreachable |
| 2715 | 0.300 | 0.596 / 1.500 | 0.0655 / 0.0367 | not reached | steering-unreachable |

## Root-cause conclusion

The circular progress frame is coherent in the observed samples: each listed
progress delta is within the existing 1.5 m continuity proof. The earliest
violated invariant is instead the live actuation join.

The tactical worker result is reused for several live 40 Hz callbacks. During
its 0.20--0.47 s age, the active Track/Follow six-state owner continues to
change steering and velocity. Advancing the prospective artifact cursor to
the live control time is correct, but its resulting steering or velocity is no
longer reachable from the actually committed predecessor. The revalidator is
therefore correctly rejecting the old plan; relaxing its physical limits would
hide the causal divergence.

The next structural boundary is a current-state prospective solve and physical
certification after tactical side selection. Async left/right results may rank
homotopies, but they cannot themselves become production execution evidence
after their initial state has diverged. No parameter change is justified by
this run.

## Verification

- Source-contract pytest: 52 passed.
- `make autoware-build`: 25 packages passed.
- Focused CTest targets: 2 passed.
- Full overlay package CTest: all 49 targets passed.

An initial command accidentally targeted the retired
`/aichallenge/build/multi_purpose_mpc_ros` tree and exposed stale binaries and
undefined symbols. It is excluded from regression evidence. The canonical
overlay built by `make autoware-build` is
`/aichallenge/workspace/build/multi_purpose_mpc_ros`; all tests there passed.
