# Design: current corpus architecture comparison

Use the existing observation-only `mpcc_architecture_compare` executable. It
rebuilds every arm from one immutable interaction snapshot and applies the same
exact trajectory adapter, wall proof, dynamic proof and terminal successor
proof.

Start with two distinct failure families from sequence 2970:

1. dynamic-obstacle refinement solve rejection;
2. post-refinement-linearization solve rejection.

If their classifications differ, expand by failure family. If all A--D arms
fail without a proof of physical infeasibility, stop local patching and inspect
a genuinely nonlinear/multi-SQP feasibility route and the upper-rank GMPCC
architecture before designing production changes.
