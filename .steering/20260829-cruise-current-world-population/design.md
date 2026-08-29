# Design: shared normal-avoidance population

## Producer replacement

The existing Follow population is already the required lower-level shape:

1. rebuild positive and negative candidates from one immutable current world;
2. retain separate persistent solver contexts per side;
3. prefer the previously certified side for continuity;
4. accept only after the common seven-state, exact wall, exact dynamic and
   terminal proof chain;
5. replace the single certified normal-plan Store.

This Slice generalizes that owner to `Cruise | Follow`.  It is active only
when the snapshot contains a current dynamic-obstacle refinement contract.
Obstacle-free Cruise continues through the ordinary canonical solve.

The rebuilt candidate keeps `execution_side_sign=0` and its original intent.
The side exists only inside candidate provenance and dynamic-obstacle rows.
Consequently the output remains a Cruise/Follow artifact and cannot enter
Overtake Mission or no-return state.

## Deletion

- remove `build_follow_escape` and `build_follow_escape_candidates`;
- rename Follow-only solver contexts, homotopy owner and population evaluator
  to normal-avoidance ownership;
- make dynamic-obstacle Cruise enter the shared population before the direct
  canonical solve;
- assert by source contract that the population is the only such path.

The neutral automatic branch remains available inside the low-level refiner
for audit/compatibility callers, but has no production caller for a current
Cruise/Follow obstacle contract after this Slice.

## Safety and timing

The population is bounded to two direct sides.  It runs inside the existing
latest-only asynchronous worker and does not add a worker or publisher.  The
preferred certified side is evaluated first, so steady-state cycles normally
retain one-solve behavior; the other side is evaluated only when the preferred
candidate cannot certify.
