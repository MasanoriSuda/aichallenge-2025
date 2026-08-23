# Validation

## Rejected Gate 1

- Run: `output/20260824-074307`
- Domain: 1 (`make dev2`)
- Result: rejected
- Earliest authority violation: decision 2410
- Evidence: complete canonical five-state solution was replaced by
  `overtake-wall-admission-hold`, producing `legacy-normal-bypass`.
- Timing evidence: callback duplicate solve repeatedly reached 4000
  iterations; async worker compute increased to approximately 50 ms.

This run is pre-fix evidence only and cannot satisfy acceptance.

## Rejected Gate 2

- Run: `output/20260824-080151`
- Domain: 1 (`make dev2`)
- Result: rejected
- Positive evidence: ShiftOut emitted certified canonical commands or explicit
  Emergency, and emitted no `overtake-wall-admission-hold`.
- Earliest remaining authority violation: decision 2405
- Evidence: `FollowPrepare/DynamicWait` lost its lateral prefix, canonical
  intent became `unknown`, and control fell through to
  `legacy-mpc-solved` / `legacy-normal-bypass`.
- Later `Rejoin` legacy output belongs to Recovery and is outside this Slice.
- Safety observation: the vehicle had already crossed the physical wall-margin
  boundary before decision 2405 and later contacted the wall. This run is not
  performance or safety acceptance evidence.

Gate-derived correction: unresolved DynamicWait now returns the existing
canonical Emergency before the legacy normal formulation block.

## Static Gate after correction

- `git diff --check`: pass
- source/deletion contract Python compile: pass
- `make autoware-build`: pass, 25 packages built
- `colcon test --packages-select multi_purpose_mpc_ros`: pass
- result after final correction: 1,739 tests, 0 errors, 0 failures, 0 skipped
- unrelated environment warning: stale
  `build/joycon_contract_guard/package.xml` result could not be opened

## Accepted Gate 3

- Run: `output/20260824-081312`
- Domain: 1 (`make dev2`)
- Result for this Slice: accepted
- ShiftOut authority traces: 21 total
  - certified canonical five-state: 9
  - explicit Emergency: 12
  - legacy normal: 0
- Pass/Return legacy normal: 0
- unresolved DynamicWait legacy normal: 0
- `overtake-wall-admission-hold`: 0
- actual-footprint static-wall contact transition: 0
- async Overtake worker during normal ShiftOut: approximately 4--8 ms
- Overtake callback aggregate near the end of ShiftOut: 7.409 ms average,
  31.822 ms maximum, one overrun; the rejected callback duplicate solve is
  absent.

The Mission exited ShiftOut through DynamicWait into Recovery because no
wall-feasible current-side lateral authority remained. Two later
`legacy-normal-bypass` traces are Rejoin/Recovery and are outside the
ShiftOut/Pass/Return promotion boundary. After Recovery, the independent
Track/Cruise production solver repeatedly exhausted 4000 iterations and
triggered a separate stuck-recovery episode; it is not acceptance evidence for
or against this Slice and must not be patched here.
