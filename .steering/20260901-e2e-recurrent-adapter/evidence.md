# Evidence

## Prerequisite data-contract correction

The first recurrent experiment was not admissible evidence: its builder copied
already-normalized `[0, 1]` scans while claiming physical metres.  Schema v2
requires `scan_unit=m`, reloads the immutable physical source arrays, and proves
their normalized identity before creating a derived sequence.  All comparisons
below use only corrected `recurrent_direct_v2` data.

The frozen PyTorch base replay agreed with the persisted runtime base outputs at
0.000105 rad aggregate MAE.  A few sparse samples reached 0.11255 rad, so the
runtime/PyTorch numerical path is not treated as bit-identical evidence.  The
adapter's required identity is instead proved within the same loaded PyTorch
base: its zero-initial correction produces exactly the base output.

## Candidate A: frozen compact-feature adapter

Checkpoint:

`checkpoints/recurrent-adapter-v2/20260901_140432/best_model.pth`

| validation subset | frozen base MAE | candidate MAE | improvement |
|---|---:|---:|---:|
| all | 0.01333 rad | 0.02197 rad | -64.8% |
| anchor | 0.00101 rad | 0.01126 rad | gate passed |
| material | 0.12999 rad | 0.12335 rad | +5.1% |

The unseen seed-2028 full-run MAE worsened from 0.00944 to 0.01842 rad.  Its
material subset improved only 3.4%, below the required 30%.  Finite/bounded and
anchor gates passed; full-validation, material and unseen gates failed.

## Candidate B: frozen base plus per-beam pressure tokens

Checkpoint:

`checkpoints/recurrent-adapter-v3/20260901_140633/best_model.pth`

| validation subset | frozen base MAE | candidate MAE | improvement |
|---|---:|---:|---:|
| all | 0.01333 rad | 0.01970 rad | -47.8% |
| anchor | 0.00101 rad | 0.00811 rad | gate passed |
| material | 0.12999 rad | 0.12947 rad | +0.4% |

The unseen seed-2028 full-run MAE worsened from 0.00944 to 0.01583 rad and its
material MAE worsened 1.4%.  Adding physical-range pressure detail did not
resolve generalization.  Finite/bounded and anchor gates passed;
full-validation, material and unseen gates failed.

## Decision

Neither adapter is eligible for runtime shadow or production.  The comparison
isolates a data/model limitation without adding another runtime fallback: the
admitted TinyLidarNet checkpoint and `fixed_lidar_brake` authority remain
unchanged.  Further model/loss tuning on the same ten sequences is stopped; the
next useful input would be broader dynamic-obstacle coverage or a different
teacher/data collection plan.

## Regression verification

```text
python3 -m pytest -q
86 passed
```
