# Requirements

## Objective

Freeze and qualify the current packaged spatial TinyLidarNet controller as the
E2E submission candidate after the recurrent-authority experiments produced no
material closed-loop benefit and failed the three-vehicle temporal-state Gate.

## Frozen production identity

- raw TinyLidarNet SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- spatial steering adapter SHA-256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`
- spatial authority: enabled, `+/-1.2 rad`
- recurrent checkpoint: absent
- recurrent authority: disabled
- longitudinal mode: `fixed_lidar_brake`
- acceleration: `0.8 m/s2`
- maximum forward speed: `4.6 m/s`

## Constraints

- Do not change a checkpoint, model architecture, steering/acceleration/speed
  bound, obstacle threshold, launch default or Gate threshold in this Slice.
- Do not inject an experiment environment variable into the packaged-default
  runs.
- Do not treat an external recurrent-shadow run as packaged-default evidence.
- Do not edit or commit `output/`, result JSON, rosbag or unrelated user files.
- A failed Gate records a submission blocker; it does not authorize a retry
  with relaxed criteria.

## Definition of Done

- Source, install-space and submission-archive artifact identities match.
- Launch/interface and controller tests pass in the development image.
- The unmodified packaged-default single-vehicle Gate passes.
- Only after single passes, the packaged-default three-vehicle d3 Gate passes.
- The submission archive contains one top-level `aichallenge_submit/` directory
  and no external recurrent artifact.
- The final evidence records accept/reject and all remaining competition risks.
