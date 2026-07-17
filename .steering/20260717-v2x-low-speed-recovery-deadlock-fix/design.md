# V2X低速回避・回復方向デッドロック修正 Design

作成日: 2026-07-17
状態: Implemented / dev3 Acceptance Failed

## 1. LowSpeedAvoidance stall watchdog

ROS非依存の`v2x_overtake_core`へ、低速状態の開始時刻・継続時間・timeoutを更新するhelperを追加する。
runtimeは前周期の確定stateが`LowSpeedAvoidance`で、実速度が閾値以下の間だけtimerを継続する。

timeout時の優先順位:

1. danger / emergency front: SafetyBrake
2. frontまたはsideあり: Follow
3. 関連車両なし: Cruise

同時にlow-speed local targetをresetし、設定cooldownまでは新規LowSpeedAvoidance候補を抑制する。
Followのside-only停止はdeliberate stopに含めないため、通常前進で離脱できなければ既存の
evidence-free stuck detectorへ引き渡される。

## 2. Recovery candidate commitment

現在はstatic rolloutがfeasibleな時点で候補をmemberへ保存するため、SUSPECT中にReverseが固定される。
保存を次のstateへ到達した場合だけに遅延する。

- `ShiftToReverse`
- `WaitReverseReport`
- `ReverseManeuver`
- `ForwardManeuver`

clearance待機中とrecoverable SafeStop中は毎周期、現在pose・wall分類・V2X配置から候補を再評価する。
actuation開始後は従来どおり固定候補のみを評価する。

## 3. Forward deadlock fallback

map footprintがclearで、選択中のReverse候補がある場合はwall分類にかかわらず最大0.6 mの
Forward Straight / Left / Rightを評価する。Front wallでは通常static rolloutがrejectするが、
wall分類が揺れても安全な候補を取りこぼさない。

Reverse corridorがblock、Forward static rolloutとforward V2X corridorがclearの場合だけ
Forwardへ切り替える。両方向blockならHoldStop / recoverable SafeStopを維持する。

## 4. Parameters

2025 AWSIM / final_ver3向けローカル実験値として次を追加する。

```yaml
v2x_low_speed_avoidance_stall_speed: 0.15
v2x_low_speed_avoidance_stall_timeout_sec: 1.5
v2x_low_speed_avoidance_stall_cooldown_sec: 3.0
v2x_low_speed_avoidance_stall_max_observation_gap_sec: 0.2
```

2026公式値ではない。

## 5. Experiment

1. baseline `output/20260717-220801`の遷移を比較対象にする。
2. pure unit test、stuck recovery suite、V2X suite、recovery footprint suiteを実行する。
3. `make autoware-build`を実行する。
4. `make dev3`を起動し、WP 134-136を含む少なくとも1周相当を観測する。
5. 各DomainについてLowSpeedAvoidance滞留、stuck state、SafetyBrake距離、MPC failureを解析する。

## 6. Rollback

- stall timeoutを十分大きくするか、LowSpeedAvoidance自体を無効化すれば旧挙動へ戻せる。
- stuck recoveryは既存の`stuck_recovery.enabled` / Domain overrideで無効化できる。
- topic、message、launch契約に変更はない。

## 7. Experiment outcome

初回dev3 run `output/20260717-225927`では、今回対象にしたWP 134〜136より前に別の
wall/rejoin failureで3台停止した。LowSpeedAvoidance stall watchdogと回復方向fallbackの
実走発火条件を満たさなかったため、unit testでの成立は確認できたがdev3受け入れ条件は未達とする。
安全なstatic / V2X rolloutがない状態で強制駆動する変更は行わない。
