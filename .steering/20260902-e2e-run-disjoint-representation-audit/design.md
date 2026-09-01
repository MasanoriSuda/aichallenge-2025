# Design

## Aggregate source discovery

Successful runs are immutable and split at extraction time.  Requiring every
physical source root to contain both train and validation would force copies or
symlinks that obscure provenance.  The recurrent builder therefore permits an
individual additional source root to contain only one split, while requiring
the aggregate discovery result to contain at least one train and one val
sequence.  Sequence identities remain globally unique.

## Causal derivative

For `speed_committed_teacher` sources, the recurrent builder inherits the
exact speed value, speed timestamp and non-negative age recorded by the raw
relabeler.  It does not resynchronize the bag.  Active-only or novelty-only
sources are rejected because their missing scans break the teacher state
history.

## Representation probe

Compare static LiDAR+speed+base features against temporal differences on the
same train and validation sequences.  The probe predicts left/neutral/right
successor correction and is diagnostic only.  Selection emphasizes held-out
material-sign recall while bounding false material corrections on neutral
states.

The resulting choice controls the next offline candidate architecture.  It
does not change the currently packaged model or ROS publisher.
