# Design

Attempt to relabel all four finalized bags with the exact executed
`speed_committed_teacher` and the strict competition report.  Store them under
a new ignored raw dataset root with `split=val`.  Derive a recurrent corpus with
`--require-executed-success` and the explicit 0.100 s speed freshness contract.
Reject a complete domain when any scan lacks causal fresh speed; do not skip a
sample from the middle of the stateful teacher sequence or relax the runtime
contract.

Evaluate these immutable checkpoints:

1. frozen production recurrent model;
2. frozen peer-augmented 64-unit model;
3. peer-augmented 512-unit capacity ablation;
4. peer run-balanced sampling candidate.

Use the same frozen base and production spatial checkpoint as prior gates and
the same 0.02 rad deployment deadband.  This Slice supplies a direct
execution-disjoint comparison; it does not prove unseen-world generalization.
