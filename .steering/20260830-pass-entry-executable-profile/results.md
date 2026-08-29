# Results

## Static verification

- `make autoware-build`: 25 packages passed.
- `ctest --output-on-failure`: 54/54 tests passed.
- `git diff --check`: passed.

The pure contract covers:

- a projected 4.73/6.0 m/s2 prefix that requires an exact successor replan,
- an unmodified 5.87/6.0 m/s2 successor that is available,
- an 8.15/6.0 m/s2 physical violation, and
- a wall-adjusted profile that remains unavailable.

## Rejected dynamic experiment

Run: `output/20260830-020321/d1/autoware.log`

The temporary implementation that allowed a projected prefix to authorize
Pass produced two `ShiftOut -> Pass` transitions.  It was rejected because:

- episode 2 reached Return and then reported
  `actual footprint intersects static wall`, and
- episode 6 remained in Pass while physical revalidation failed, then
  reported `actual footprint wall margin violated`.

The experiment proved that a repaired short prefix is not an immutable
Pass/Return successor certificate.

## Final Dynamic Acceptance

Run: `output/20260830-022414/d1/autoware.log`

- `Idle -> ShiftOut`: 1
- `ShiftOut -> Pass`: 0
- `Pass entry physical gate held`: 1
- physical footprint contact reports: 0

The observed gate hold was a different strict branch,
`static wall clamp exceeds lateral acceleration limit`; the projected-prefix
reason did not occur in this short run.  The failure-first unit test therefore
remains the direct verification of that classification.  No projected prefix
was promoted to Pass.

This run exposed a separate pre-existing failure: the published ShiftOut
artifact lost its terminal contingency certificate and production fell from
`terminal-contingency-unavailable` to `retained-proof-unavailable`, Stop and
Recovery.  It is deliberately not patched in this slice.

## Frozen architecture comparison

The Pass failure snapshot
`000000003254-53fd1752047f5dc2-pass-side-positive-wall-refinement-coupled-solve-rejected`
from the rejected dynamic experiment was replayed without changing the solver,
wall, obstacle or terminal-successor proof chain.

- persistent A: solver rejected
- stateless B: rejected
- rough-right C, transition stage 18 / ahead stage 20: accepted bundle
- offline-right D, transition stage 19 / ahead stage 20: accepted bundle
- bounded production G: rejected

Per the frozen exit classification, this is a candidate-generation defect at
that snapshot, not physical infeasibility and not a solver-tolerance issue.
Because the snapshot is already in Pass/no-return execution, the accepted
opposite homotopy is evidence for an earlier candidate-comparison deficiency;
it is not authority to switch sides after no-return.  The next slice must trace
why that current-world terminal transition was absent before commit.
