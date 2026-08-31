# Results: terminal Stop / normal-resume architecture audit

## Observed phenomenon

Decision 2451 entered certified Stop after the Cruise normal candidate reported
`terminal-contingency-unavailable`.  The next callback had no executed normal
clock and evaluated an older candidate from cursor zero; its expected pose was
already about 1.30 m from the current control pose.  Subsequent
`steering-unreachable`, stuck-Recovery and prolonged Stop are downstream of
this first normal-authority loss.

## Frozen A--D comparison

All arms used interaction fingerprint `883737710184574622`, the same wall
grid, `d2` prediction tube, physical footprint, actuator bounds and terminal
Stop contract.

| arm | result | decisive evidence |
|---|---|---|
| A recorded Cruise | rejected | maximum iterations; dynamic lateral row stage 4, normalized violation 240.267 |
| B current-world positive | rejected | maximum iterations; dynamic lateral row stage 4, normalized violation 301.581 |
| B current-world negative | rejected | maximum iterations; effective-progress row stage 8, normalized violation 0.184833 |
| C smooth positive lattice | rejected | no certified member among 210 schedules |
| C smooth negative lattice | **accepted** | transition stage 18, ahead stage 20, exact wall/dynamic/Stop proofs accepted |
| D positive multi-SQP | rejected | no certified member among 210 schedules |
| D negative multi-SQP | rejected | fixed depth-three outer iteration did not preserve the depth-zero accepted C member |

The accepted C bundle ended at 4.274 m progress and 1.124 m/s with 0.123 m
minimum lateral reserve.  It retained a certified terminal Stop suffix.

## Root-cause classification

**A/B fail and C succeeds: candidate-generation defect.**

The current Cruise/Follow production population contains only one direct
candidate per side.  At this snapshot, immediately committing either complete
side disjunct is not numerically/physically executable.  A late, smooth
current-world right-side schedule can continue decelerating while preserving a
future avoidance route and passes the unchanged proof chain.  Therefore:

- the initial Stop was not proven physically mandatory;
- retaining the old pre-Stop Mission path is not the repair;
- changing clearances, solver tolerances or timeouts is not justified;
- later `steering-unreachable` is a symptom of the earlier normal candidate
  population hole.

The D failure does not override the C certificate.  D starts from a fixed
depth-three outer iteration and can regress an already certified depth-zero
candidate; production must always prefer the first fully certified artifact.

## Existing patch relationship

The Stop successor and normal execution-ledger reset exposed the problem but
did not create it.  A certified Stop remains a valid contingency.  The missing
piece is an executable normal candidate before that contingency becomes the
only authority.  Adding another Stop resume rule would only hide this upstream
candidate hole.

## Structural repair selected

Unify bounded topology generation for Overtake and Cruise/Follow:

1. direct side remains the first candidate;
2. derive a steering-reachable smooth/physical diagonal from the measured
   current state;
3. retain a bounded midpoint/encounter-boundary topology;
4. evaluate candidates in anytime order and accept the first complete proof;
5. keep Cruise/Follow identity neutral and do not acquire Overtake authority.

This should be implemented by sharing the existing bounded population logic,
not by adding a decision-2451 special case.

## Verification

- `make autoware-build`: 25 packages passed.
- `test_mpcc_stateless_maneuver`: 29/29 passed.
- `test_mpcc_architecture_comparison`: 34/34 passed.
- package CTest: 59/59 passed.
- frozen A--D replay: C negative accepted; command exit status 0.

## Next dynamic check

- decision-family `terminal-contingency-unavailable` should no longer occur
  when a certified smooth normal-avoidance candidate exists;
- normal candidate logs must identify direct versus steering-reachable versus
  midpoint/boundary topology;
- certified Stop must still take authority when every bounded candidate fails;
- no new authority, lease, timeout, clearance or tolerance path may appear.
