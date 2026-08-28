# Design: recorded prepared-suffix replay

The recorder already serializes `assembly_request`, but the replay loader
currently discards it.  Extend the v2 interaction loader so the architecture
audit owns all three layers:

1. immutable semantic/world source;
2. exact final assembly request; and
3. assembled QP plus its recorded warm start.

An offline CLI chooses an elapsed time inside the captured horizon, linearly
interpolates the recorded primal solely as a deterministic latest-state probe,
builds the time-aligned prepared suffix, and solves it cold.  It also runs the
full semantic pipeline from the original source for an approximate compute-cost
comparison.

The interpolated probe is not physical evidence and cannot promote authority.
It answers only whether the 20-stage transform is computationally plausible
and whether the preserved refined QP remains numerically solvable.
