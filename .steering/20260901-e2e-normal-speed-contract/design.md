# Design

```text
dagger_aggregate_v2 sequence
  + immutable physical scans.npy
  + source bag /localization/kinematic_state
  -> nearest timestamp synchronization
  -> longest contiguous interval within 50 ms
  -> normal_anchor_recurrent_v1
```

The dedicated schema contains scans, speeds and both timestamp streams.  Its
only target semantics are `frozen_base_steering_correction_equals_zero`.
`steers.npy` and direct steering labels are deliberately absent so the dataset
cannot be silently reused as a direct-policy teacher.

Generated arrays stay outside git.  Source paths, label source, topics, message
types, rejection counts and synchronization statistics are persisted in each
sequence metadata and the root manifest.
