# Evidence

> Superseded: a later audit found that this generated dataset stored scans
> normalized to `[0, 1]` while declaring a 30 m physical-range contract.  The
> candidate metrics below are not valid architecture evidence.  Production was
> unchanged, so no runtime rollback is required.

## Derived dataset

Source dataset:

`aichallenge/ml_workspace/tiny_lidar_net/dataset/precontact_residual_base_v4`

Derived generated dataset:

`aichallenge/ml_workspace/tiny_lidar_net/dataset/recurrent_direct_v1`

- 10 immutable sequences: 7 train, 3 validation;
- 34,735 accepted samples;
- `/localization/kinematic_state` `nav_msgs/msg/Odometry` speed;
- nearest timestamp synchronization limited to 50 ms;
- maximum accepted delta 47.305 ms;
- temporal holes are not bridged: only the longest contiguous accepted interval
  from each source run is retained;
- unseen seed 2028 (`output/20260901-130837`) remains validation-only.

## Candidate 1: run-balanced direct GRU

Generated checkpoint:

`checkpoints/recurrent-direct-v1/20260901_134940/best_model.pth`

| validation subset | frozen base MAE | candidate MAE | improvement |
|---|---:|---:|---:|
| all | 0.01333 rad | 0.17601 rad | -1220.4% |
| anchor | 0.00101 rad | 0.16570 rad | -16318.4% |
| material | 0.12999 rad | 0.27361 rad | -110.5% |

The output collapsed toward approximately -0.27 to +0.04 rad.  The root cause
was partly sampling: equal run mass over-weighted short failure prefixes whose
material-action fractions are 60--87%, compared with 9--22% in long runs.

## Candidate 2: base distillation then successor fine-tuning

Generated checkpoint:

`checkpoints/recurrent-direct-v2/20260901_135142/best_model.pth`

| validation subset | frozen base MAE | candidate MAE | improvement |
|---|---:|---:|---:|
| all | 0.01333 rad | 0.12557 rad | -842.0% |
| anchor | 0.00101 rad | 0.11790 rad | -11581.8% |
| material | 0.12999 rad | 0.19824 rad | -52.5% |

Unseen seed 2028 full-run MAE was 0.11688 rad versus the frozen base's 0.00944
rad.  The base-only distilled checkpoint also produced 0.14977 rad aggregate
MAE, proving that successor weighting was not the only failure.

All candidates remained finite and bounded, but every accuracy/generalization
gate failed.  Production and runtime shadow were therefore unchanged.  The
unit mismatch, rather than the model, is the earliest proven cause.

## Corrected physical-metre rerun

The schema was advanced to version 2 and now requires `scan_unit=m`.  The
builder reloads each immutable source `scans.npy`, proves
`physical_scans / max_scan_range_m == parent_loader_scans`, and only then writes
the recurrent sequence.  Old schema-v1 data is rejected rather than silently
reinterpreted.

Corrected generated dataset:

`aichallenge/ml_workspace/tiny_lidar_net/dataset/recurrent_direct_v2`

- 10 immutable sequences: 7 train, 3 validation;
- 34,735 samples;
- maximum speed synchronization delta 47.305 ms;
- validation physical LiDAR minima 1.410, 0.889 and 0.563 m, with 30 m maxima;
- production/runtime remained frozen throughout the rerun.

Corrected checkpoint:

`checkpoints/recurrent-direct-v3/20260901_140328/best_model.pth`

| validation subset | frozen base MAE | candidate MAE | improvement |
|---|---:|---:|---:|
| all | 0.01333 rad | 0.08031 rad | -502.5% |
| anchor | 0.00101 rad | 0.07422 rad | -7253.8% |
| material | 0.12999 rad | 0.13801 rad | -6.2% |

On unseen seed 2028, full-run MAE was 0.06911 rad versus the frozen
base's 0.00944 rad; material MAE was also 15.4% worse.  The candidate stayed
finite and bounded, but full-validation, material-improvement, anchor and unseen
gates all failed.  This is the valid architecture result; no runtime shadow or
production change is permitted.

## Regression verification

```text
python3 -m pytest -q test/test_recurrent_policy.py test/test_extract_data_contract.py
20 passed
```

```text
python3 -m pytest -q
86 passed
```
