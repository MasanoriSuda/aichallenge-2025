# Row-certified Follow shadow design

## Root cause

Persistent OSQPは混合単位QPをglobal infinity normで終了判定する。course progressが
10--20 mのscaleを持つため、acceleration、curvature、zero velocityなど小scale rowの
誤差が各rowの既存toleranceを超えてもglobal tolerance内となり得る。

下流のexecution normalizationはrow-wise toleranceで再認証するため、solverはsolved、
executionはrejectedという二重契約になっていた。

## Chosen correction

`ConstraintCertificationPolicy`をsolver construction時に固定する。

- `GlobalInfinityNorm`: 現行挙動。legacyとproductionを変更しない。
- `RowWiseExecution`: OSQP internal scalingでterminationを評価し、active-set polish後の
  primalを既存row-wise residual reportで必ず認証する。

row-wise認証失敗はresultを返さず、`stage=row_certificate`として明示する。下流で
clamp、repair、tolerance緩和は行わない。

## Migration boundary

このSliceではFollow shadow contextだけを`RowWiseExecution`へ接続する。動的Gateを通過後、
Follow authority昇格Sliceでproduction extended solverへ同じcontractを移し、shadow専用の
差分を解消する。

## Experiment result

この案は`output/20260823-142145`で不採用とした。scaled terminationはOSQP内部の
scaled空間では早く終了したが、元の物理単位での最大違反を0.02--0.03へ増加させた。
polishもこの不一致を解消しなかった。

row-wise rejectをsolver failureへ移しただけでaccepted率は悪化したため、実装は全て
revertした。production solverへは接続していない。

頻出した`row=270`はN=20のextended QPで最初のcurvature-rate constraintに対応する。
次の根本原因はtermination flagではなく、mixed-unit formulationのnondimensionalizationと、
steering-rate可行領域の余白不足として扱う。
