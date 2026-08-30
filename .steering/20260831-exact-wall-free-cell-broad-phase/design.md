# Design: exact wall free-cell broad phase

`sample_footprint()` currently evaluates oriented-box intersection for every
cell in the footprint AABB and only then reads occupancy.  Almost all cells on
the racing surface are free, so exact geometry dominates every swept sample.

Cell occupancy is a sound broad phase: a free cell can never contribute a
contact, while occupied and unknown cells must still use the existing exact
box/cell intersection.  Reorder the two checks:

1. read the cell state;
2. skip `Free` immediately;
3. run the unchanged exact geometry for non-free cells only.

This preserves fail-closed semantics, checked poses, swept interpolation,
contact indices and reject reasons.  It requires no cache or provenance rule.

Dynamic evidence shows lower mean evaluation cost but a remaining wall-proof
tail.  The next Slice may add an immutable non-free-cell integral index so a
wholly free footprint AABB can be rejected in O(1).  That is deliberately not
folded into this Slice: the simple ordering change is independently reviewable
and its effect is already measured.
