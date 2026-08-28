# Design

## Root cause

One `target_id` field was implicitly serving two different concepts:

1. the persistent tactical encounter target of Follow/Overtake semantics;
2. the current obstacle whose predicted tube generated QP rows.

For Cruise, concept 1 is intentionally absent while concept 2 can be active.
The solver snapshot therefore assembled target-dependent rows but its sealed
context had no matching target provenance.  The replay verifier correctly
rejected that incomplete problem.

## Correction

Extend `MpccProblemContext` with a dedicated constraint identity:

- active flag;
- obstacle ID;
- observation generation;
- constraint-side sign (`0` for stay-behind, `-1/+1` for a pass homotopy).

These values are included in the immutable problem fingerprint and the
architecture snapshot.  Completeness is all-or-none.  The producer populates
them from the same selected target provenance and dynamic-obstacle contract
that created the stage rows.  The worker compares replay-world provenance to
these fields, not to the semantic Mission target.

## Non-goals

- permitting a newer obstacle generation to masquerade as the source world;
- retaining a solution by age alone;
- changing candidate geometry, solver cadence or physical margins;
- solving the later current-world revalidation policy in this Slice.

If generation mismatch remains after the missing identity is fixed, the next
Slice must determine whether world capture is non-atomic.  It must not weaken
the new fingerprint contract.

## Snapshot schema boundary

New architecture failure snapshots use schema v2 because the dedicated
dynamic-obstacle constraint identity participates in the immutable problem
and interaction fingerprints.  Existing v1 artifacts remain valid for exact
QP replay.  They are deliberately rejected for physical A/B/C/D interaction
replay: reconstructing the missing identity from the semantic target or the
latest replay world would hide the very provenance mismatch this Slice fixes.
