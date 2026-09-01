# Design

Candidate5 is deliberately tested instead of candidate6.  Candidate6 preserves
normal behavior but learned only 2.6% of the new correction.  Candidate5 learned
53.1% and passed the three-lap single-vehicle gate, so it is the only current
candidate capable of falsifying the contact-learning hypothesis.

No checkpoint is copied into the package.  `TINY_LIDAR_CKPT_PATH` overrides all
four containers for this one run.  If it fails the peer gate, no further global
fine tuning is attempted; the accepted base policy and the corrective policy
must be separated, for example as a learned residual with zero-output normal
anchors.
