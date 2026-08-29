# Results: bounded mid-horizon diagonal candidate

## Causal verification

The regression test first failed with the old production population:

- expected mid candidate full-side stage: 9;
- actual old candidate full-side stage: 2.

After replacing the abrupt candidate, the same frozen snapshot produced:

```text
arm=production-left-g stage=solver-rejected candidate_source=direct-side candidate_count=3
arm=production-right-g stage=accepted candidate_source=mid-physical-diagonal
lattice_transition=0 lattice_ahead=9 candidate_count=2
terminal_progress=17.175 terminal_velocity=7.78118
lateral_reserve=0.00000572226
```

The unchanged SQP and exact proofs therefore continue to reject the invalid
left side while certifying the previously omitted right-side temporal
homotopy. This falsifies a blanket proof relaxation and supports candidate
timing as the root producer for decision 1272.

## Static verification

- `make autoware-build`: passed, 25 packages.
- focused `test_mpcc_stateless_maneuver`: passed, 17 tests.
- full package CTest: passed, 54/54.
- interface review: no ROS topic, service, launch, Domain or result-schema
  contract changed.

## Dynamic Gate

- `output/20260829-155302`: excluded; AWSIM stopped at Unity memory setup and
  never published state or odometry.
- `output/20260829-155509`: valid two-vehicle run.

In the valid run, the observed current worlds admitted `direct-side` before a
mid candidate was needed. Two direct candidates passed solver, physical and
dynamic proof and reached Gate A. No `mid-physical-diagonal` artifact was
published in this run, so live adoption of that candidate remains a targeted
scenario check rather than a claimed race acceptance.

The candidate population stayed bounded at three and ran in the asynchronous
worker. The control callback had no sustained overrun attributable to this
change. Uncertified population results remained `selected=0`; a certified
direct result alone reached `selected=1`.

## Separate residual failures

The run exposed two pre-existing, independently classified failures:

1. episode 1: `ShiftOut -> Recovery`, `locked target stale or lost`;
2. episode 2: `ShiftOut -> Recovery`, `actual footprint wall margin violated`.

Episode 2 correctly entered Emergency when no certified Rejoin artifact was
available. These failures did not use the new mid candidate and are not hidden
by this Slice. They become frozen evidence for the next root-cause audit; no
new resume rule, lease, tolerance or clearance was added here.

## Review findings

No blocking code-review finding remains. The obsolete
`EarliestPhysicalDiagonal` enum/log path and `first + 2` production construction
were deleted rather than retained as a fallback. The only remaining risk is
dynamic coverage: a live world that requires the mid candidate must still
demonstrate end-to-end publication and execution.
