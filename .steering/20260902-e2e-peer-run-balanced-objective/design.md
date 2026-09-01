# Design

The augmented train split contains one 6,180-sample historical run and four
domains (17,496 samples total) from one final-world run.  Natural sampling
therefore assigns most optimization steps to one correlated world.

Add an opt-in sampler whose per-chunk weight is:

`1 / chunks_in_same_source_run`

After normalization, every immutable source run owns equal total probability
mass while sequences inside that run retain their empirical chunk proportion.
The existing natural and sequence-balanced modes remain unchanged.

The grouping identity comes only from
`metadata.outcome_certificate.source_run_id`.  Directory names and mutable
paths are not accepted as identity fallbacks.
