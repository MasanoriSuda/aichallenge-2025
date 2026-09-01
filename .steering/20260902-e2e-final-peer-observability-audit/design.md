# Design

Reuse the existing nearest-neighbour observability audit.  The only tool
change is making its recurrent speed freshness contract explicit, consistent
with training/evaluation.  Keep 50 ms as the fail-closed default and pass
100 ms for the executed final-world teacher corpus.

Sample each sequence deterministically, compare material teacher states to
successful zero-correction normal states, and measure overlap relative to
cross-run normal p50/p95 distance.  A high overlap in physical geometry means
the label/observation contract is ambiguous.  A high overlap only after the
projected adapter indicates representation compression.
