# Results: proof-authoritative wall pose

## Root cause

The lag and heading intervals generated while sampling a wall pose bucket were
both promoted to hard state boxes in the racing QP.  They were not physical
wall constraints: the exact nonlinear trajectory and swept footprint proof
were already the physical authority.  On frozen ShiftOut fingerprint
`11478535197026802675`, applying both sampled pose boxes emptied the useful
affine search region while omitting both produced a trajectory accepted by all
unchanged exact proofs.

## Direct versus feasibility-first replay

The audit separated the two possible explanations:

- joint omission with Phase-I: certified, terminal progress `14.875 m`,
  terminal velocity `7.07755 m/s`;
- joint omission with the racing objective directly: certified with the same
  terminal state;
- production formulation after deletion: certified with the same terminal
  state.

Therefore production does not need a second feasibility solver, a fallback or
a solver-setting change.  The invalid hard pose boxes can simply be removed.

Across the six-snapshot wall corpus the new production arm produced four
certified bundles.  One trajectory remained rejected by the exact swept-wall
proof and one QP remained numerically rejected.  An independent dynamic
counterexample, fingerprint `12107242968934788374`, remained rejected by the
unchanged timed-obstacle proof.

## Production change

Normal seven-state wall refinement keeps:

- the lateral/progress convex wall corridor;
- swept wall rows;
- the exact nonlinear trajectory proof;
- the exact swept physical-wall proof;
- the timed dynamic-obstacle proof;
- the terminal successor proof.

It no longer applies sampled lag and heading pose-bucket intervals as hard
normal-authority state boxes.  The historical variants remain reachable only
from the offline architecture comparison.  Runtime telemetry now records
`lag_pose_box` and `heading_pose_box`, so the deleted authority is observable.

## Validation

- focused tests: `2085` passed, `0` failed;
- `make autoware-build`: `25` packages built successfully;
- frozen wall corpus: four certified, one exact-wall rejection, one numerical
  rejection;
- dynamic run: `output/20260829-133704`.

The dynamic run exercised production artifacts with
`lag_pose_box=0/heading_pose_box=0` and unchanged exact proofs.  It did not
complete Overtake.  The first ShiftOut authority gap was:

1. fresh ShiftOut accepted and published;
2. later current-world solve rejected/staled after exact swept-wall rejection;
3. the last published artifact intermittently failed `progress-lift-rejected`;
4. one-cycle canonical Emergency braking occurred.

The same sequence already exists in pre-change run `output/20260829-122448`:
one ShiftOut, five source handoff rejections, one progress-lift rejection and
five ShiftOut Emergency publications.  The new run had one ShiftOut, five
source handoff rejections, two progress-lift rejections and four ShiftOut
Emergency publications.  This is not evidence that pose-box deletion created
the failure.

Replaying the new frozen failure fingerprint `9845010060330222052` showed that
persistent, stateless, rough/lattice and production candidates all failed the
same racing solve.  It is a separate candidate/globalization or single-SQP
case, not a reason to restore the disproved pose boxes.

## Outcome

Accept the structural deletion.  Do not tune clearances or solver settings.
The next Slice must classify fingerprint `9845010060330222052` and the
`new-solution unavailable -> retained progress-lift rejection -> Emergency`
chain with the architecture escape-hatch process.
