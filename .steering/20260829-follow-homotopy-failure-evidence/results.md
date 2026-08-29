# Results: Follow homotopy failure evidence

## Verification

- `make autoware-build`: passed, 25 packages.
- `ctest -R test_mpcc_architecture_snapshot`: passed, 9 tests.
- full `multi_purpose_mpc_ros` CTest: passed, 54 tests.
- Bounded dynamic run: `output/20260829-152340`.

The dynamic run did not reproduce a two-sided Follow failure.  Follow selected
and published a certified preferred positive candidate at decision 1089.
This is a valid non-reproduction rather than evidence for changing Follow.

The same run did prove the recorder change: opposite ShiftOut homotopies wrote
independent bounded artifacts instead of suppressing one another.  Captures
included negative physical-proof and solve failures and positive solve
failures.

## Frozen comparison

Snapshot:

`output/20260829-152340/d1/mpcc_architecture_snapshots/000000001272-1e0b2c02a78399a9-shiftout-side-negative-physical-proof-physical-proof-rejected/snapshot.yaml`

Using the exact frozen world and unchanged solver settings:

- persistent A failed exact physical proof at stage 309;
- target-bound A2 failed identically;
- stateless left B failed the relinearized solve;
- stateless right B reproduced the exact A physical-proof failure;
- production left/right G evaluated three bounded candidates per side and
  rejected all;
- wall restoration H reproduced A;
- rough right C produced certified candidates;
- physical-diagonal right F produced many certified candidates, including
  transition stage 11 / ahead stage 15 with terminal progress 17.2447 m,
  terminal speed 7.674 m/s and positive exact lateral reserve.

## Root-cause classification

The frozen case is not physical infeasibility and is not fixed by rebuilding
the same persistent Mission problem.  A/B fail while C/F succeed under the
same world snapshot and seven-state refinement.  The earliest invalid
producer is therefore the bounded production candidate generator: its three
direct-side candidates omit a physically certified diagonal transition which
the audit lattice can generate.

The downstream physical proof is behaving correctly.  Relaxing it, changing
OSQP tolerances, adding a timeout, or retaining the rejected Mission would
mask the upstream candidate omission.

## Scope of this Slice

This Slice changes evidence identity only.  Production authority, solver
settings, clearances, timing and candidate selection remain unchanged.  The
next Slice may replace the deficient production candidate population with a
bounded diagonal population, but must not copy the exhaustive offline lattice
into the live control callback.
