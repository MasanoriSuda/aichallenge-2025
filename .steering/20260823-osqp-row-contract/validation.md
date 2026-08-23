# Validation: OSQP row-contract root-cause audit

## Static validation

- `make autoware-build`: 25 packages successful
- `colcon test --packages-select multi_purpose_mpc_ros`: 38 test targets
- `colcon test-result --verbose`: 1635 tests, 0 errors, 0 failures

Failure-first testでは、1000 m級のprogress rowと0.1級の小単位rowが同居すると、
小単位rowが自身の許容値を90倍以上超えても旧global tolerance比較を通るケースを
再現した。

## Dynamic comparison

| Run | Candidate | Decisions | Execution/row reject | Max-iter | Wall proof reject | Callback overrun | Stuck |
|---|---|---:|---:|---:|---:|---:|---:|
| `20260823-081219` | baseline | 3910 | 26 | 1 | 0 | 0 | 0 |
| `20260823-093759` | row normalization | 3589 | 4 | 1 | 7 | 0 | 4 |
| `20260823-095004` | row normalization + dual rebase | 5559 | 5 | 4 | 16 | 1 | 0 |

Latest runのrow rejectはすべてwarm solveで、row 210/213/215/264だった。5状態、
N=20のlayoutではinput box rowsに対応し、主にaccelerationまたはvirtual progress
velocityの境界である。拒否周期の次周期はcold solveでcertifiedへ復帰した。

## Decision

Reject. Row-contract mismatchの頻度は大きく改善したが、物理wall証明拒否、
max-iteration、overrunを悪化させた。壁用fallbackや閾値変更を同じSliceへ追加せず、
candidate source/testを削除した。

残った上流課題は、5状態MPCCのnondimensionalizationとfirst-stage/wall execution
contractの統合監査である。
