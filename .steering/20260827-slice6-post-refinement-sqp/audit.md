# Audit

## Status

Implementation complete; full dynamic closure is blocked by a separately
identified fresh/retained authority hole and remains open in the next Slice.

## Dynamic evidence

`output/20260827-185648/d1/autoware.log`:

- decision 1778 admitted canonical ShiftOut;
- physical wall refinement initially solved and certified;
- final fresh evaluations then reported `exact=invalid-lateral-bounds`;
- decision 1844 retained proof reported `proved_stages=0` and
  `current_stage_valid=0`;
- decision 1844 published typed Emergency with
  `retained-proof-unavailable`;
- the episode later lost its live corridor and entered wall Recovery.

The run has no missing wall-clearance contract and no wall-refined QP failure,
so the previous wall-contract Slice remains valid.  The first new defect is
the missing post-refinement nonlinear correction.

## Prototype falsification

`output/20260827-191743` and `output/20260827-192725` falsified an
unconditional extra-solve design:

- the additional post-refinement QP itself reached maximum iterations;
- transporting the previous same-problem input prefix reduced provenance
  ambiguity but did not remove that failure;
- therefore "a refinement happened" is not sufficient reason to solve again.

The implementation now asks the production exact physical replay first and
only performs bounded same-problem SQP corrections after an actual proof
rejection.

## Verification

- `make autoware-build`: 25 packages passed.
- focused shadow CTest: passed.
- source deletion/authority contract: 64 passed.
- full package test: 1981 tests, 0 errors, 0 failures, 0 skipped.
- `make dev2`: `output/20260827-194608`.

In the final run, physically refined normal solutions report
`post_refinement_linearization ... count=0/proof=1/1`.  This demonstrates that
already valid refined candidates are accepted without the prototype's extra
solve.  The run contains no `post-refinement relinearized QP rejected` and no
`physical-proof-rejected` event.

The remaining `invalid-lateral-bounds` messages belong to the separate
overtake pre-entry exact-swept path, not to final normal-authority artifact
construction.  Later, decision 3279 loses authority when no fresh asynchronous
result is available in the same cycle and retained current-world continuation
also rejects.  That fresh/retained timing hole is the next root-cause Slice;
it must not be hidden by weakening the physical proof here.

## Rollback

Baseline commit: `ce9a21d`.
