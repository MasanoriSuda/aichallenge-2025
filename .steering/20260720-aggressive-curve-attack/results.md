# Results

## 実装結果

- soft curve用の内／外entryとは別に`hard_entry_allowed`を追加した。
- dev3ではhard curve認識後も、対象側gap成立時に内差し／外回りShiftOutを開始できる。
- ShiftOut / Pass中のgap width、gap time、reachable gap一時欠落を最大0.5秒holdする。
- holdは最後の有効gap時刻を延長せず、明示WP禁止、cooldown、EmergencyBrake、target jumpを隠さない。
- gap条件を入口2点/0.5秒、継続1点/0.3秒へ分離した。
- hard entryとgap holdの状態をV2X state/debug logへ追加した。

## dev3攻め側設定

- 入口前方距離4.0 m、継続1.8 m、prepare 3.0 m
- guard/line横加速度6.0 m/s2
- ShiftOut 4.0 m、Return 6.0 m
- ShiftOut closing 2.0 m/s、未ラッチPass 1.0 m/s
- lateral clearance latch 0.75 m
- Return rear-clear 2.0 m、確認0.10秒、curve cooldown 0.30秒
- moving front hard center 2.05 m、target distanceは5.0 mを維持
- wall clearance 0.1 m、inflated obstacle gap 0.2 mを維持

## 維持したhard gate

- 明示禁止WP
- wall/body境界とpass-side gap
- locked target ID、position jump
- EmergencyBrake / SafetyBrake
- solver failure / odometry timeout / NaN / Inf

## 検証

- `make autoware-build`: 成功、25 packages finished。
- 追加対象test: 16/16成功。
  - active gap-loss hold 2件
  - outer curve 7件
  - inner curve 7件
- `test_v2x_overtake_core`: 110/110成功。
- `git diff --check`: 成功。

## dev3ログ確認項目

- `inner_hard_entry=1`または`outer_hard_entry=1`から`Idle -> ShiftOut`へ入る。
- gap欠落時に`gap_hold=1`となり、`hold_rem`が0.5秒以内で減少する。
- hold中にtarget jump、EmergencyBrake、明示WP禁止が出た場合は継続しない。
- `OvertakeLine: ShiftOut -> Pass -> Return -> Idle`まで完了する。
- `SafetyBrake`、`MPC control failed`、接触、curve cooldown再試行回数を前runと比較する。

dev3実走は未実施。2025由来AWSIMシミュレーション予選向けの比較設定であり、実車値ではない。
