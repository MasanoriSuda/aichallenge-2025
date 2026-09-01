# Design

## Evidence leading to this Slice

The two-channel scan-delta model can fit the final d1/d2 failure tails, but only
with severe normal-state leakage.  Increasing anchor/gate regularization removes
the leakage by driving the correction back to zero.  Existing successful
precontact data came from one deterministic four-peer run, while both newest
hard failures came from another run.

This is a run-distribution generalization problem until disproved.  Adding an
RNN before collecting independent dynamic-obstacle trajectories would increase
capacity without adding evidence.

## Dataset split

- seed 2027 successful NPC teacher: train;
- seed 2028 successful NPC teacher: validation;
- retain the existing run-level train/validation split;
- retain the d1/d2 hard failure prefixes only in train;
- retain independent normal data as a separate leakage gate.

The generated datasets, checkpoints and bags remain ignored artifacts.  The
steering evidence records immutable run paths, metrics and checkpoint hashes.

## Admission order

1. Finish/stall admission for both teacher runs.
2. Relabel with the frozen production base and precontact teacher.
3. Train the already implemented `scan_delta` residual.
4. Evaluate hard d1/d2 tails, historical anchors, full validation and
   independent normal data.
5. Stop before closed loop on any failure.

## Outcome

Both seed-disjoint teacher runs passed, but the frozen scan-delta residual did
not generalize.  It fit only the strongly signed failure tails and leaked into
normal/anchor states.  See `evidence.md`.  This rejects data-coverage-only and
loss-weight-only explanations; production remains unchanged.
