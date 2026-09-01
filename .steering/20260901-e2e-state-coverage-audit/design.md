# Design

Use the first AWSIM penalty as the earliest externally observed failure
boundary, mapped from the first 1 m/s motion sample.  Retain ten seconds before
and 0.5 seconds after that boundary; uniformly cap query samples without
changing their order.

Compare each query with:

1. candidate3's actual train/validation corpus;
2. admitted `LidarPrecontactTeacher` sequences, including seed-disjoint runs.

Use two independent feature spaces: deterministic binned physical ranges and
the frozen TinyLidarNet `fc3` embedding.  Define the reference scale from
cross-sequence nearest neighbours so temporally adjacent samples in one run do
not create an artificially tiny novelty threshold.

For action ambiguity, take at most one nearest observation from each teacher
sequence.  Opposite material steering signs among close observations are an
aliasing signal, not an automatic architecture verdict.
