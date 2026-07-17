# 壁接触予防・再合流安定化 Design

作成日: 2026-07-17
状態: Implemented / Experiment Follow-up Required

## 1. Front hazard hold

ROS非依存の`v2x_overtake_core`にdeadline型のhazard hold helperを追加する。runtimeは
SafetyBrake相当のfront dangerを観測したtarget IDを保存し、設定時間内に同じtargetが一時的に
front/side判定から外れてもSafetyBrakeを維持する。現在messageで明確にrear-clearと確認できた場合は
期限前でも解除する。

暫定値:

```yaml
v2x_front_hazard_hold_enabled: true
v2x_front_hazard_hold_sec: 1.0
v2x_front_hazard_rear_clear_distance: 4.0
```

## 2. LowSpeedRejoin preflight

Recovery safety snapshotへ`rejoin_forward_static_clear`を追加する。normal MPCの先頭操舵符号と絶対値を
使い、現在poseから設定距離までForward Straight / Left / Rightのswept footprintを評価する。

```yaml
stuck_recovery:
  rejoin:
    static_lookahead_m: 0.8
    retry_on_blocked_path: true
```

current footprintにcontactがある場合は`rejoin_unsafe` SafeStopを維持する。current clearで
preflightだけblockedなら`STOP_AND_CONFIRM -> CHECK_CLEARANCE`へ戻し、既存の距離・step・V2X上限内で
追加候補を再評価する。

## 3. Heading-aware retreat

current footprint clear時は、static safeなReverse 3候補についてrollout終端yawを計算し、
`abs(current_heading_error + yaw_delta)`が最小の候補を選ぶ。同値はStraight / Left / Right順とする。
contact中は既存のcontact reduction最大化を変更しない。

## 4. Experiment

1. core unit test、recovery footprint testを実行する。
2. `make autoware-build`を実行する。
3. `make dev3`を起動し、旧停止時刻62秒より長く走行する。
4. D2のhazard hold、D1のrejoin preflight、全車のWP/速度/recovery stateを解析する。

## 5. Rollback

- hazard holdは`v2x_front_hazard_hold_enabled=false`で無効化できる。
- rejoin retryは`retry_on_blocked_path=false`で従来のSafeStopへ戻せる。
- static/V2X fail-closed条件は変更しない。

## 6. Experiment outcome

`output/20260717-232948`では、前方危険targetを一時的に失った直後のCruise復帰は再発せず、
D3は停止したD1の後方でSafetyBrakeを維持した。LowSpeedRejoinの前進preflightもD1前方の
wall collisionを検出し、新規contactを作る前に駆動を禁止した。

ただし、D1の安全な前進rolloutがなく、D3がD1の後退corridorを塞いだためD1はSafeStopした。
D3はD1後方でSafetyBrake、後から到達したD2もD3後方でSafetyBrakeとなり、D2のStartから約79秒後には
3台停止した。
次段は通常走行中のwall departure抑制を優先し、それでも閉塞が残る場合に限り、後続車が安全な空間を
作るcooperative yieldを別ステアリングで設計する。
