# Learned steering residual requirements

## Objective

Preserve the admitted production TinyLidarNet bit-for-bit while learning the
pre-contact lateral correction that a global fine-tune and an fc4-only
fine-tune could not represent without a regression.

## Invariants

- The production checkpoint and default launch behavior do not change in this
  slice.
- Runtime lateral authority remains ML.  No deterministic gap teacher may be
  enabled in a submission mode.
- The runtime residual target is the steering difference between
  `LidarPrecontactTeacher` and the frozen production TinyLidarNet base steering
  evaluated on the same scan.  `LidarGapTeacher` is retained only as diagnostic
  provenance because it is not present in the production composition.
- Normal states are explicit zero-residual anchors; corrective states may not be
  trained as a positives-only dataset.
- Every residual dataset sequence records source bag, split, both teacher
  identities, base checkpoint identity and target statistics.
- A residual checkpoint is not connected to ROS until held-out correction and
  normal-anchor gates pass.

## Definition of Done

1. Dataset generation stores auditable base, reference-teacher and runtime
   residual targets.
2. A dedicated residual dataset/model/trainer rejects malformed provenance and
   reports corrective and zero-anchor metrics separately.
3. Unit tests cover target identity, split isolation, weighting and zero-output
   initialization.
4. A frozen all-peer dataset is split by domain, trained and evaluated without
   changing production.
5. Runtime integration is attempted only when offline gates justify it.
