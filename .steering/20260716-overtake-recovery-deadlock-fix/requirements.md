# Overtake Recovery Deadlock Fix Requirements

作成日: 2026-07-16
更新日: 2026-07-16
状態: Complete

## 目的

明示的なV2X追い越しラインがsolver failureでRecoveryへ遷移した後、速度上限が現在速度まで縮小して0 m/sへ固定され、後続車もSafetyBrakeで長時間停止するデッドロックを解消する。

## Baseline evidence

対象run: `output/20260716-215126`

- P2は1784206316.9907に`ShiftOut -> Recovery`へ遷移した。
- solverは直後に復帰したが、P2はwp51で約48秒間`Cruise`、前方車なし、solver failureなしのまま0 m/sだった。
- P2のRecoveryは1784206372.1423まで約55秒継続した。
- P1は停止したP2を約3.78 m前方に検出し、約49秒間SafetyBrakeで停止した。

## 機能要件

### R-RECOVERY-01: 再加速可能な速度上限

- Recovery速度上限は設定値`v2x_overtake_recovery_velocity`を使用する。
- solver Recoveryでも速度上限を現在速度でclampしない。
- 速度上限は直接速度指令ではなくMPCの上限として適用し、既存の加速度上限を維持する。

### R-RECOVERY-02: 正しいphase走行距離

- `current_speed * phase_elapsed`を走行距離として使用しない。
- control周期ごとに`max(0, signed_forward_speed) * dt`を積算する。
- phase遷移時に積算距離と観測時刻をresetする。
- clock rollback、非finite値、長い観測gapは積算せず、stall連続判定もresetする。

### R-RECOVERY-03: stallと総時間timeout

- Recovery中に`v2x_overtake_recovery_stall_speed`以下が連続`v2x_overtake_recovery_stall_timeout_sec`続いたらRecoveryを解除する。
- Recovery全体が`v2x_overtake_recovery_timeout_sec`を超えたら解除する。
- SafetyBrake/EmergencyBrakeは既存どおり最優先で、Recovery解除によって安全停止を上書きしない。
- 距離または横偏差による既存の正常完了条件を維持する。

### R-RECOVERY-04: solver failure cooldown

- solver failure起因のRecovery終了後は`v2x_overtake_solver_cooldown_sec`の間、新しい明示的追い越しラインを開始しない。
- cooldownはV2XのSafetyBrake/Follow/Cruise判断や通常trajectory追従を停止させない。
- cooldown中も別の安全停止条件は従来どおり有効とする。

### R-CONFIG-01: 暫定設定

```yaml
v2x_overtake_recovery_velocity: 3.0
v2x_overtake_recovery_stall_speed: 0.15
v2x_overtake_recovery_stall_timeout_sec: 1.0
v2x_overtake_recovery_timeout_sec: 5.0
v2x_overtake_recovery_max_observation_gap_sec: 0.2
v2x_overtake_solver_cooldown_sec: 2.0
```

- 数値は2026公式上限ではなく、現行dev3向けの暫定値とする。
- speed/timeout/gapはfiniteかつ正、cooldownはfiniteかつ非負とする。
- stall timeoutはRecovery全体timeout以下とする。

### R-LOG-01: 診断

- Recovery debugに経過時間、積算距離、stall時間、速度上限を出す。
- distance、lateral、stall、timeoutの終了理由を区別する。
- solver cooldownによる追い越しライン抑止を状態変化またはthrottle付きで確認できるようにする。

## 非機能要件

- `/control/command/control_cmd`、odometry、trajectory、V2X topicの名前と型を変更しない。
- P1/P2/P3別trajectory、Boost設定、stuck recovery設定を変更しない。
- 評価基盤`aichallenge_system/`を変更しない。
- P1のSafetyBrake距離を弱めて症状を隠さない。
- 実車適用前にdev3シミュレータで検証する。

## 受け入れ条件

- 0 m/sでRecovery速度上限が0にならない。
- 速度列を積分した距離と、最終速度×経過時間を区別できるunit testが通る。
- 1秒stallまたは5秒timeoutでRecoveryが終了する。
- solver failure後2秒間は同じ追い越しラインを再開しない。
- SafetyBrake中はRecovery解除後も停止優先を維持する。
- `make autoware-build`、対象unit test、`make dev3`が成功する。
- dev3でP1/P2が同じ地点に10秒以上停止し続けない。
