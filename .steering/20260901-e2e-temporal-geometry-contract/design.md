# Design

Keep `precontact_residual_base_v4` as the original admitted source.  Add a
separate generated `competition_failure_teacher_v1` source containing:

- NPC d1 and peer d1 as train sequences;
- peer d3 as a validation-only sequence.

`build_recurrent_dataset.py` accepts one primary and repeated additional source
roots.  It discovers and validates every source identity before writing output,
then records the owning source root in each sequence as before.  This avoids
copying or symlinking old immutable arrays into a synthetic aggregate.

The output remains schema v2 because every per-sequence array and unit contract
is unchanged.  The manifest adds `additional_source_dataset_roots`; existing
single-root consumers remain valid.
