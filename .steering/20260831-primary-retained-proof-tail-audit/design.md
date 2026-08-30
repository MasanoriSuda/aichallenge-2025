# Design: primary retained proof tail audit

The retained evaluator already measures pre-continuation, continuation build,
continuation proof, terminal build, terminal dynamic proof and terminal wall
proof.  Transition logging is throttled and can miss the exact slow decision.

Append the aggregate fields to the existing slow production-join warning.
No additional proof work is introduced and runtime data remains diagnostic.

The first bounded run isolated `continuation_proof` but that region still
contained three different operations.  The same typed runtime structure was
therefore refined into non-overlapping:

- measured-to-control delay wall proof;
- continuation reconstruction and dynamic-obstacle proof;
- continuation exact swept-footprint wall proof.

The second run showed that wall proof is the dominant region.  The next Slice
must inspect repeated wall-grid traversal and proof reuse/provenance.  It must
not weaken clearance, shorten the certified horizon, or replace exact proof
with an age lease.
