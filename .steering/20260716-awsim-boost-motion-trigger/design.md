# AWSIM Boost Motion Trigger Design

作成日: 2026-07-16
更新日: 2026-07-16
状態: Implemented / dev3 verified

## 方針

公式Boost I/Oと既存の1回限り/no-retry状態機械は維持し、Boost eligibilityだけを `Start seen` から `Ready prepared + actual forward motion` へ変更する。

`/awsim/state=Start` の意味は変更しない。Startはdomain start-speed window、start-grid grace、recovery sessionなどの既存利用者へ従来どおり通知し、Boost guard内部だけがReadyとodometryから別のmotion triggerを作る。

## 現行問題

現行guardは次の順序でpulseを許可する。

```text
/awsim/state=Start
  -> start_seen=true
  -> normal control + fresh status
  -> [1.0] / [0.0] pulse
```

count開始時の実際の順序は次である。

```text
Ready
  -> start-grid SafetyBrake解除
  -> 物理発進（ego >= 0.1 m/s）
  -> 6〜8秒走行
  -> 車両別Start
  -> Boost pulse
```

このためpulse自体は成功しても、スタートダッシュではなく走行中Boostになっている。

## 変更後シーケンス

```text
AWSIM state          control / odometry        Boost guard
    Ready -------------------------------> motion watch prepared
                              ego < 0.1 --> wait
                    healthy command publish --> eligible
                             ego >= 0.1 ----> motion edge captured
    fresh status ---------------------------> validate remaining / isBoosting
                                              PublishPulse
    /awsim/cmd [1.0] <---------------------- node
    /awsim/cmd [0.0] <---------------------- node
    status changed ------------------------> Confirmed

    Start -------------------------------> existing race event only
```

Readyは発動許可ではなく監視準備である。pulseは前進速度閾値と正常control publishが同時に成立した後に限定する。

## State machine

既存の使用済み状態を維持し、発動前を明示的に分割する。

```text
Disabled
  └─ enabled + SIM                         -> Armed

Armed
  ├─ Ready                                 -> AwaitingMotion
  ├─ low-speed Start fallback              -> AwaitingMotion
  └─ Finish                                -> session closed

AwaitingMotion
  ├─ forward speed < threshold             -> AwaitingMotion
  ├─ healthy speed >= threshold            -> MotionDetected
  ├─ reverse / invalid / stale odometry     -> wait or fail-safe
  └─ duplicate Ready/Start                  -> AwaitingMotion（epoch不変）

MotionDetected
  ├─ healthy + fresh status + remaining     -> PulseSent
  ├─ temporary status/control block         -> MotionDetected（短時間だけwait）
  ├─ timeout / max trigger speed exceeded   -> LaunchExpiredSpent
  └─ SafetyBrake / recovery                 -> block; timeout後はspent

PulseSent
  ├─ isBoosting / remaining decrease        -> Confirmed
  └─ confirmation timeout                   -> UnconfirmedSpent

Confirmed / UnconfirmedSpent / LaunchExpiredSpent
  ├─ duplicate state/status/control         -> same（no pulse）
  └─ Finish -> Spawned                      -> Armed
```

`LaunchExpiredSpent` はBoost残数を物理的には消費していないが、スタート用途としては使用済み扱いにする。遅延発動より、発動見送りを安全側とする。

## Pure guard API

ROS非依存の `StartDashGuard` にmotion trigger入力を追加する。

候補API:

```cpp
enum class Trigger
{
  AwsimStart,
  FirstForwardMotion,
};

struct TriggerContext
{
  bool control_enabled;
  bool normal_command_published;
  bool failsafe_active;
  bool v2x_safety_brake_active;
  bool solver_fallback_active;
  bool reverse_or_recovery_active;
  double forward_speed_mps;
};

Evaluation evaluate(const TriggerContext & context, TimePoint now);
```

guardの責務:

- Ready / Start / Finish / SpawnedによるBoost session状態管理。
- signed forward speedのfinite・閾値・上限検証。
- motion edgeとtrigger timeoutをsteady clockで管理。
- status freshness、残数、Boost中、使用済み判定。
- pulse、確認、timeout、no-retry判定。

nodeの責務:

- odometry由来の符号付き前進速度を渡す。
- control publish成功後にguardを評価する。
- V2X SafetyBrake、MPC fallback、forced stop、recovery inhibitをcontextへ集約する。
- `PublishPulse` のみROS topicへ変換する。
- 状態変化をログへ出す。

## StateEventとの分離

`StartDashGuard::on_awsim_state()` が返す `StartEntered` はBoost以外にも使用されている。

- `domain_start_epoch_`
- start-grid grace arm
- manual reset speed window
- recovery session

したがって `Ready` で `StartEntered` を返す変更は禁止する。Boost用には `ReadyEntered` eventを追加するか、guard内部の `ready_seen` とphaseだけを更新する。既存Start eventの時刻・重複排除は変更しない。

## Motion detection

- sourceは車体前方軸のsigned odometry speedとする。
- `abs(speed)` はReverseを発車と誤認するため使用しない。
- `speed >= motion_speed_threshold_mps` を最初に満たしたsteady timeを1回記録する。
- hysteresisは1セッション1回latchにより不要。ノイズ確認が必要なら連続時間を追加するが、初回実装では発動遅延を増やさない。
- Ready後に既にspeed閾値を超えている場合は、`speed <= max_trigger_speed_mps` の範囲だけ最初の評価をmotion edgeとみなす。

## Trigger window

motion edge後、通常は同じ40 Hz control cycleでpulseする。statusが更新境界にある場合だけ最大0.5秒待つ。

```text
motion_detected_at <= now <= motion_detected_at + timeout
AND forward_speed <= max_trigger_speed
```

timeoutはsteady clockのみで比較し、ROS timeと混在させない。未来statusやclock rollbackは失効とする。

## Safety arbitration

Boost評価は通常command publish成功後に置く現行順序を維持する。

```text
odom / input validation
  > operator stop / forced stop
  > V2X SafetyBrake
  > solver fallback
  > recovery action / reverse / recovery inhibit
  > normal command publish
  > launch Boost trigger
```

Readyでstart-grid guardがSafetyBrakeを解除する前にBoostを送らない。発車速度が閾値を超えていても、その周期のV2X stateがSafetyBrakeならpulseしない。

## Config

```yaml
awsim_boost:
  enabled: true
  domain_enabled:
    1: true
    2: true
    3: true
  mode: start_once
  trigger: first_forward_motion
  motion_speed_threshold_mps: 0.1
  max_trigger_speed_mps: 1.0
  motion_trigger_timeout_sec: 0.5
  status_timeout_sec: 0.5
  confirmation_timeout_sec: 2.0
```

`trigger: awsim_start` は比較試験とrollback用に残す。既存configにtriggerがない場合の互換既定値は、移行時の意図しない挙動変更を避けるなら `awsim_start`、このリポジトリのconfigには明示的に `first_forward_motion` を設定する。

## Logging

状態変化時だけ次を記録する。

- `AWSIM Boost motion watch prepared: state=Ready`
- `AWSIM Boost launch motion detected: speed=..., ready_elapsed=...`
- `AWSIM launch Boost pulse published: motion_delay=..., remaining_before=...`
- `AWSIM launch Boost skipped: motion trigger timeout`
- `AWSIM launch Boost inhibited: SafetyBrake / solver fallback / recovery / stale status`
- 既存のConfirmed / UnconfirmedSpentログ

runtime比較用にReady、motion、pulseの3時刻を同一ログclockで取得できるようにする。

## 予定変更ファイル

| ファイル | 変更内容 |
|---|---|
| `include/.../awsim_boost_start_dash.hpp` | trigger enum、motion phase/context、config追加 |
| `src/awsim_boost_start_dash.cpp` | Ready準備、motion edge、timeout/upper-speed判定 |
| `src/mpc_controller_cpp.cpp` | signed speedと安全状態の入力、時刻差ログ |
| `config/config.yaml` | first-forward-motion設定と暫定閾値 |
| `test/test_awsim_boost_start_dash.cpp` | motion trigger・SafetyBrake・timeout・互換test |
| `docs/spec/mpc-integration.md` | 発動epochとrollback設定を更新 |

topic/type契約は変わらないため、`docs/interface/participant-interface.md` は内容確認のみとし、不要な契約変更を行わない。

## Test design

### Pure unit

- Ready前、Ready静止中、閾値直前はpulseなし。
- Ready + threshold crossing + healthy/statusで1回pulse。
- motion後statusが短時間遅れてもwindow内ならpulse。
- timeout/max speed後は `LaunchExpiredSpent` でpulseなし。
- Start fallbackは低速時だけ準備し、高速遅延Startでは失効。
- SafetyBrake、solver fallback、forced stop、reverse/recovery中はpulseなし。
- duplicate Ready/Startでmotion timeやtimeoutを延長しない。
- pulse後の確認/no-retryとFinish -> Spawned rearmを維持。
- `awsim_start` triggerで既存test互換を維持。

### Runtime

- `make dev3` でReady、初回0.1 m/s、pulse、確認の時刻をd1〜d3から抽出する。
- `pulse - first_motion <= 0.25 s` を確認する。
- pulse high/low各1回、remaining減少1回を確認する。
- Boost無効化runと0〜10秒の速度・加速度を比較する。
- collision、wall、SafetyBrake、solver failureを確認する。

## Rollback

1. `awsim_boost.trigger: awsim_start` で旧Start triggerへ戻す。
2. `awsim_boost.enabled: false` またはDomain overrideでBoost自体を無効化する。
3. rollback時もlegacy `use_boost_acceleration` は有効化しない。

## Interface impact

- ROS topic/type: 変更なし。
- Domain構成: 変更なし。
- AWSIM state文字列: 変更なし。
- control command: 変更なし。
- result JSON / submission: 変更なし。
- participant package内部のBoost trigger configと状態機械だけを変更する。
