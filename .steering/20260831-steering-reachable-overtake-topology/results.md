# Results

## Root-cause classification

Frozen source:

`output/20260831-124927/d1/mpcc_architecture_snapshots/000000001141-1f493d8bb2bb9e50-shiftout-side-negative-post-refinement-linearization-physical-dynamic-sqp-audit-solve-rejected/snapshot.yaml`

The unchanged architecture comparison classified the failure as candidate
generation:

- persistent A and direct stateless B failed exact wall proof;
- rough/lattice C accepted the left branch at transition stage 6 and ahead
  stage 20;
- offline D accepted the same topology;
- the physical-diagonal audit F independently accepted full-side stage 6;
- the old bounded production G offered direct side, fixed midpoint stage 9
  and late exact, and certified none of them.

Therefore this was `A/B fail, C succeeds`, not a Mission-lifecycle defect,
single-SQP limitation, certificate mismatch or physical infeasibility.

## Implemented correction

The bounded stateless population now contains a
`SteeringReachablePhysicalDiagonal` member.  Its full-side stage is derived
from the current measured steering, side steering limit, steering-rate limit,
yaw-response time and immutable stage durations.  For the frozen source this
is stage 6:

```text
abs(+0.366519 - -0.172680) / 0.731707 + 0.130000 = 0.867 s
```

The population remains bounded and evaluates, in order, direct side,
steering-reachable diagonal, distinct midpoint diagonal, and finite-boundary
or late-exact topology.  A duplicate reachable/midpoint stage is emitted only
once.  Seven-state SQP, authority, wall/opponent/terminal certificates and all
configuration values are unchanged.

## Verification

- `make autoware-build`: 25 packages passed.
- package CTest: 59/59 passed; 2318 tests, zero failures or errors.
- focused tests verify stage-6 reachability, four-member ordering, midpoint
  deduplication and finite-boundary retention.
- post-change replay of the frozen source certified production-left with the
  unchanged single SQP and exact proofs:

```text
arm=production-left-g
stage=accepted
candidate_source=steering-reachable-physical-diagonal
lattice_transition=0
lattice_ahead=6
sqp_depth=0
bundle=1
solve_ms=64.9147
```

Older frozen snapshots were replayed, but their complete current solver/proof
pipeline no longer reproduces the historical acceptance after later model
changes.  Preservation of midpoint and late-exact population is therefore
covered by focused structural tests rather than an invalid cross-version
dynamic claim.

Bounded `make dev2` validation used `output/20260831-131649`.  No stale or
uncertified publication attributable to the new candidate appeared.  Normal
callback windows were generally below the 25 ms period, with a few later
overruns during Recovery.  The run did expose a separate pre-existing failure:
an accepted direct-side frozen Mission was held for about 1.3 seconds, then
the actual footprint violated wall margin and ShiftOut entered Recovery.  The
steering-reachable candidate was not selected in that episode because direct
side had already been certified.  This is evidence for a distinct
post-admission geometry/lifecycle audit, not a reason to alter this candidate
generation correction.

## Next frozen defect

Freeze the earliest callback where the accepted direct-side path changes from
current-world viable to wall-bound.  Compare rebuilding its exact trajectory
from the current world against retaining the published geometry.  Do not add
a Mission resume rule, grace period, wall-margin change or Stop fallback.
