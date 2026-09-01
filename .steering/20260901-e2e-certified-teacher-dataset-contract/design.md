# Design

Introduce one supervision-provenance library rather than duplicating admission
rules in each dataset builder.  The library validates the strict competition
report against local immutable artifacts and emits a compact outcome
certificate.

The teacher relabeler accepts `--competition-analysis` and
`--require-executed-success`.  Strict mode refuses to write a sample until the
teacher was the unique runtime steering owner and the run completed all laps
with zero penalty and zero post-start stall.  The certificate hash participates
in `sequence_id` generation.

Recurrent derivation copies the complete certificate instead of reconstructing
it.  Its opt-in strict flag makes absence or alteration of the certificate an
input-contract error.  This preserves backward readability while preventing a
new certified training pipeline from silently mixing historical unproven
labels.

No steering model is trained or promoted in this slice.  A second independent
successful teacher run is still required for a run-disjoint validation split.
