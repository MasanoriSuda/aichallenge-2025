# Evidence

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
gate failed.  Production and runtime shadow were therefore unchanged.

## Verification

```text
python3 -m pytest -q test/test_recurrent_policy.py test/test_extract_data_contract.py
16 passed
```

```text
python3 -m pytest -q
82 passed
```
