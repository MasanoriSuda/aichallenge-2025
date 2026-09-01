# Design

## Why this is a structural comparison

The rejected residual variants all converted a run into independent examples
and reduced the angular scan through a stride-32 CNN.  Two and eight-frame
inputs learned memorized failure tails but did not generalize policy activation.

End2Race provides a closer published reference for the actual task: preserve
per-beam LiDAR pressure, condition on ego speed, maintain a recurrent hidden
state and train complete overtaking sequences.  This Slice adopts those
principles without copying its F1TENTH-specific dimensions or reported gates.

## Dataset

A builder consumes an already admitted `precontact_residual_base_v4` sequence
and its immutable source bag.  It copies the exact admitted scans and direct
successor steering labels, then nearest-neighbor synchronizes odometry speed.

Required output:

- `scans.npy`: metres, unchanged from admitted source;
- `speeds.npy`: absolute longitudinal speed in m/s;
- `steers.npy`: direct pre-contact-teacher steering in rad;
- original scan timestamps and matched speed timestamps;
- per-sample synchronization deltas;
- metadata linking source dataset ID, source bag and source label provenance.

The builder rejects samples beyond 50 ms.  It may not silently reassign a run's
train/validation split.

## Model

- learnable monotonic sigmoid pressure transform per LiDAR beam;
- small MLP speed embedding;
- single-layer unidirectional GRU;
- bounded direct steering decoder;
- loss at every valid timestep in each fixed-length chunk.

The first comparison uses 64-step chunks (roughly the eight-second scale of the
published reference at this dataset's sampling rate).  Chunk boundaries reset
hidden state during training; evaluation runs each complete sequence in order.

The first run-balanced experiment exposed a label-distribution defect: short
failure prefixes contain 60--87% material actions and dominated much longer
nominal runs.  The controlled follow-up therefore first distils the frozen base
with sample-proportional chunks, then fine-tunes the successor teacher with a
2x material weight.  This changes only the offline training comparison.

## Offline admission

The candidate must simultaneously:

- improve material teacher MAE by at least 30% over the frozen base;
- keep anchor MAE at or below 0.03 rad for this direct-policy comparison;
- avoid worsening full validation MAE against the frozen base;
- improve or preserve the unseen seed-2028 run;
- remain finite and bounded over every complete validation sequence.

These limits are intentionally separate from residual leakage gates because a
direct policy cannot have exact zero error relative to another learned policy.

## Resulting decision

The initial comparison is superseded by a subsequent unit audit.  The derived
builder copied scans after the parent loader had normalized them by 30 m, while
the recurrent metadata and model treated them as physical metres.  The recorded
metrics remain useful symptoms, but they cannot reject the architecture.  A new
dataset identity with an explicit `scan_unit=m` contract was therefore generated
and the experiment repeated before making a runtime decision.

The corrected comparison still failed every accuracy/generalization gate other
than finite bounded output.  It therefore rejects the from-scratch recurrent
policy on the admitted data, without changing production or runtime shadow.
