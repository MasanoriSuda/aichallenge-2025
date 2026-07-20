# Results

## 実装

- soft curveで未ロックの場合、内側gapを第一候補にする。
- 内側gapが不成立なら既存`v2x_overtake_try_both_sides`で外側を再評価する。
- `inner_entry`成立時だけstart-curve、completion、soft-curve入口guardを緩和する。
- hard curve内では新規開始せず、同じlocked targetと内側gapが残る`inner_hard`時だけ継続する。
- 明示WP禁止、cooldown、EmergencyBrake、SafetyBrake、wall clearanceは維持する。
- 外回り判定と直線追い越しは従来どおり残す。

## 設定

```yaml
v2x_overtake_inner_curve_entry_enabled: true
v2x_overtake_inner_curve_hard_continuation_enabled: true
```

省略時既定値は両方とも`false`。

## 検証

- `make autoware-build`: 成功、25 packages finished。
- curve判定gtest: 16/16成功。
  - inner curve 6件
  - outer curve 6件
  - soft curve continuation 4件
- package全体test: 19 test targets中18成功、1件は既存trajectory契約で失敗。
  - `PathCoreCircular.RemovesOneEndpointFromConfiguredFinalVer3Trajectory`
  - 現在の`traj_mincurv.csv`が閉路終点の重複を持たず、テストが期待する1点削除が発生しないため。
  - イン差し実装・追加テストとは独立。

## dev3確認項目

- `reason=inner curve entry`かつ`inner_entry=1`でShiftOutへ入る。
- hard apexで`inner_hard=1`を維持する。
- `OvertakeLine: ShiftOut -> Pass -> Return -> Idle`まで完了する。
- 内側gap消失時にRecoveryへ戻る。
- `SafetyBrake`、`MPC control failed`、接触回数が外回りのみのrunより増えていないか比較する。
