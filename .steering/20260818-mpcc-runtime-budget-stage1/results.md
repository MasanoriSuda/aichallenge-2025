# Results

## 実装結果

- command/safety監視は40 Hzのまま維持した。
- lateral receding horizonへ、同一WP・同一Mission・同一制約・短時間だけのvalidated
  evaluation reuseを追加した。既定refresh intervalは0.10秒。
- first feasible QPを保持するconditional RTIを追加した。tight bound、非線形defect、
  大曲率だけ2回目を実行し、12 msを過ぎた周期では開始しない。
- physical wall envelope cacheを0.025 rad / 1 cm conservative bucket、16384件LRUへ変更した。
- `Overtake horizon schedule`と`MPCC RTI-SQP`へfresh/reuseとskip理由を追加した。

## 静的検証

- `git diff --check`: 成功
- `make autoware-build`: 成功（25 packages）
- 対象test:
  - `test_mpcc_progress`: 17/17成功
  - `test_v2x_overtake_core`: 707/707成功

## 次の実走で比較する値

直前run `output/20260818-094456`をbaselineとし、同じdev条件で次を比較する。

- `MPC callback timing`のShiftOut/Pass average、max、overruns/min
- `Overtake horizon schedule`のfresh/reused比率
- `MPCC RTI-SQP`のattempts、skip_condition、skip_deadline
- physical wall cacheのhit率、entry数、scanned pose数
- Pass完遂数、wall/solver Recovery、接触、lap time

目標はShiftOut/Passのdeadline超過を減らしつつ、Pass完遂率とhard-fault検出を悪化させないこと。
core solver thread/process分離は、このA/Bで25 ms超過が残る場合のStage 2とする。
