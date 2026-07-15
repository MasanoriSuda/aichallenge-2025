# V2X Start Grid SafetyBrake Design

作成日: 2026-07-16
更新日: 2026-07-16
状態: Implemented / dev3 startup verified（追加safety regression pending）

## 方針

現行のV2X FSMと `SafetyBrake` 優先順位は維持し、スタートグリッド用猶予の時間管理だけをレースセッションへ正しく結び直す。

静止グリッドに対する通常の停止距離判定は短時間抑制するが、相対接近から算出される `EmergencyBrake` は抑制しない。これにより、正常スタートでは後車も発進でき、前車が発進しない異常ケースでは再び停止できる構造にする。

## 現行問題

現在の猶予判定は次の構造になっている。

```cpp
if (!std::isfinite(first_v2x_behavior_eval_sec)) {
  first_v2x_behavior_eval_sec = now_sec;
}
const bool start_grid_grace_active =
  start_grid_grace_time > 0.0 &&
  now_sec - first_v2x_behavior_eval_sec < start_grid_grace_time;
```

この起点はAWSIMのレース開始と無関係である。

```text
odometry / V2X ready
  -> first behavior evaluation
  -> grace timer starts
  -> Grounded / Ready待機中に5秒経過
  -> grace expires
  -> Start時の静止前車をinside stopping distanceと判定
  -> SafetyBrake / limit=0 / solver fallback
```

さらに現行判定は `suppress_start_grid_stop_behavior` を `FrontRiskLevel::EmergencyBrake` と `has_danger_vehicle` の両方へ適用している。猶予の起点を直すだけでは、猶予中の実接近まで抑制する可能性が残る。

## Interface compatibility

### 維持する契約

- `/awsim/state`: `std_msgs/msg/String`
- `/v2x/vehicle_positions`: 現行 `v2x_msgs` 契約
- `/control/command/control_cmd`: 最終制御出力
- Domain 0: AWSIM管理面
- Domain 1..N: 車両Autoware
- `control_method=mpc`
- result JSON、`output/latest/`、提出tar.gz

### 変更しない領域

- `aichallenge/workspace/src/aichallenge_system/`
- AWSIMバイナリと起動引数
- 評価orchestratorの責務
- Boost command/state machine
- trajectory/localization/topic remap

## 設計概要

```text
/awsim/state callback
  -> normalize state
  -> existing race-session tracker
       Ready          : StartGridGraceGuard.prepare()（duration未消費）
       accepted Start : StartGridGraceGuard.arm(ROS time)
       session boundary / manual reset ready : StartGridGraceGuard.clear()

control / V2X behavior cycle
  -> detect front/side vehicles
  -> compute front risk
  -> StartGridGraceGuard.evaluate(context, now)
       static_stop_suppression
       phase/reason
  -> EmergencyBrake? -------- yes -> SafetyBrake
  -> static danger and not suppressed? -> SafetyBrake
  -> otherwise Follow/Cruise/Overtake
```

## StartGridGraceGuard

ROS依存を持たない小さな状態判定として切り出すことを第一候補とする。既存ファイル構成とのバランスから同一translation unit内のpure classとして開始してもよいが、ROS callbackやpublisherを直接持たせない。

実装状態:

```cpp
enum class Phase
{
  Disabled,
  WaitingForStart,
  Prepared,
  Grace,
  Expired,
};
```

候補入力・出力:

```cpp
struct StartGridContext
{
  bool simulation;
  bool has_front_vehicle;
  bool has_side_vehicle;
  bool front_vehicle_stationary;
  bool front_risk_emergency;
  double ego_speed_mps;
  double front_distance_m;
};

struct StartGridDecision
{
  StartGridPhase phase;
  bool suppress_static_stop_behavior;
  const char * reason;
};
```

guardの責務:

- nodeから渡された受理済みStartとsession clear eventからepochを管理する。
- Start後の経過時間を計算する。
- 重複Startでepochを更新しない。
- config無効、非simulation、時刻異常では抑制しない。
- start-grid contextが成立する間だけ静止停止車向け抑制を返す。
- EmergencyBrakeそのものは判定・抑制せず、呼び出し側が常に優先する。

## セッション状態

```text
Disabled
  └─ grace_time=0                    -> Disabled

WaitingForStart
  ├─ Ready                           -> Prepared
  ├─ accepted Start (Ready欠落時)    -> Grace(epochを1回だけ記録)
  └─ その他                          -> WaitingForStart

Prepared
  ├─ evaluate                        -> static-grid contextだけsuppression有効
  ├─ duplicate Ready                 -> Prepared
  ├─ accepted Start                  -> Grace(epochを1回だけ記録)
  └─ Finish/Spawned/Grounded/reset   -> WaitingForStart

Grace
  ├─ duplicate Start                -> Grace(epoch維持)
  ├─ elapsed >= grace_time          -> Expired
  ├─ Finish/Spawned/Grounded/reset   -> WaitingForStart
  └─ clock invalid/rollback         -> Expired（抑制なし）

Expired
  ├─ duplicate Start                -> Expired
  └─ new session boundary           -> WaitingForStart
```

通常の `Spawned -> Grounded` はWaiting状態を維持し、`Ready`でPreparedへ進む。Start後にSpawned/Groundedへ戻った場合は前sessionをclearし、同一状態の重複通知では再armしない。

## SafetyBrake優先順位

変更後の優先順位は次とする。

```text
odometry/non-finite/control/solver/collision fail-safe
  > front-risk EmergencyBrake
  > start-grid static-stop suppression
  > stopped-vehicle inside stopping distance
  > LowSpeedAvoidance / Follow / Overtake / Cruise
```

重要な変更点:

1. `FrontRiskLevel::EmergencyBrake` には `!suppress_start_grid_stop_behavior` を掛けない。
2. start-grid suppressionは静止車向け `has_danger_vehicle` と停止車向けLowSpeedAvoidanceだけへ作用させる。
3. 前車が動かず後車だけが加速した場合、相対速度とrequired decelの上昇でEmergencyBrakeへ入り、衝突前に停止する。
4. 通常走行、猶予終了後、単独走行では現行判定をそのまま使う。

## Start-grid context

猶予時間内という理由だけで全V2X車両を無視しない。

初期候補:

- `phase == Grace`
- 複数車両配置を示す `has_side_vehicle` がある
- Prepared/Grace中に対象前車を停止車閾値以下で初回確認し、target IDを1台だけlatchする
- latchした前車はmoving-front閾値以下の発進立ち上がりまで同じグリッド車として扱う
- `front_risk != EmergencyBrake`
- 入力がfinite/fresh

現行d2ではd3がfrontかつ別車両がsideとして検出されているため、このcontextで静止距離由来の `SafetyBrake` だけを抑制できる。

公式スタート配置はWIPのため、waypoint ID、固定座標、Domainごとの固定順序、3.42 mなどの観測値を条件へ埋め込まない。

## Clock設計

V2X behaviorの `now_sec` とStart epochは同じclock domainで比較する。

第一候補:

- `/awsim/state == Start` callbackでnodeのROS clock値を取得する。
- behavior cycleへ同じROS clock由来の `now_sec` を渡す。
- session reset時にepochを破棄する。
- `now < epoch`、非finite、clock jump検知時は猶予をExpiredにして抑制しない。

steady clockを選ぶ場合はStart callbackとbehavior評価の両方をsteady clockへ統一する。既存のROS time秒とsteady time pointを差し引かない。

## MPC node integration

現行 `awsim_state_callback()` には次が既に存在する。

- state文字列のtrim/lowercase正規化
- `race_started_`
- Boost guardのsession event
- `domain_start_epoch_`

このcallbackからstart-grid guardへ正規化済みstateと同一clock時刻を渡す。Boost guardや `domain_start_epoch_` の責務は変更しない。

V2X behaviorへは次のいずれかで状態を渡す。

1. 推奨: `StartGridGraceGuard` をMPC/V2X behavior側が保持し、node callbackから明示的なsession eventを渡す。
2. 代替: nodeが算出した `start_grid_grace_active` とsession epochをbehavior評価入力へ渡す。

環境変数やglobal変数から暗黙にStart状態を読む設計は避ける。

## Logging

状態変化時だけ次をINFO/WARNで記録する。

- `Start grid grace armed: duration=...`
- `Start grid static stop suppressed: front=..., side=..., distance=...`
- `Start grid grace expired`
- `Start grid grace cleared: session boundary`
- `Start grid suppression overridden: front risk emergency, required_decel=...`
- `Start grid grace rejected: invalid/rollback clock`

既存V2X debugの `grace` fieldは維持し、可能なら `grace_reason` またはphaseを追加する。40 Hzで同一ログを連打せず、状態変化またはthrottleを使う。

## Config

既存値を維持する。

```yaml
mpc:
  v2x_start_grid_grace_time: 5.0
```

実装後のdev3計測で5秒が過大/不足か評価する。値の変更はepoch修正と同じcommitで安易に行わず、A/Bログを根拠に別途判断する。

hard emergency専用距離が必要になった場合の候補:

```yaml
mpc:
  v2x_start_grid_hard_min_distance: TBD
```

既存front-riskで安全を満たせるなら追加しない。固定グリッド車間から値を逆算しない。

## 予定変更ファイル

| ファイル | 変更内容 |
|---|---|
| `multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp` | Start event接続、猶予起点、SafetyBrake優先順位、ログ |
| `multi_purpose_mpc_ros/include/.../start_grid_grace.hpp` | pure guardを分離する場合の状態・判定 |
| `multi_purpose_mpc_ros/src/start_grid_grace.cpp` | pure guard実装を分離する場合 |
| `multi_purpose_mpc_ros/test/test_start_grid_grace.cpp` | session/clock/emergency unit test |
| `multi_purpose_mpc_ros/CMakeLists.txt` | guard library/testを分離した場合のtarget |
| `multi_purpose_mpc_ros/config/config.yaml` | コメントの意味更新。値変更はruntime根拠がある場合のみ |
| `docs/spec/mpc-integration.md` | Start epoch、抑制範囲、EmergencyBrake優先を正本化 |

実際の分割は既存CMakeとテスト構造を確認して最小変更にする。

## Test設計

### Pure unit test

1. Start前に100秒評価してからStartしても猶予がフルに残る。
2. Start直後、静止front + side contextではstatic stop suppressionが有効。
3. 重複Startを受信してもepochが変わらない。
4. 猶予時間ちょうどでExpiredになる。
5. Finish/resetでWaitingForStartへ戻る。
6. 次セッションのStartで1回だけ再armする。
7. clock rollback/NaN/Infでは抑制しない。
8. grace time 0では常に抑制しない。
9. EmergencyBrake入力では猶予中でもSafetyBrakeが優先される。
10. side contextなしの通常停止車では従来判定を維持する。

### Behavior regression test

- d2相当: front=3.42 m、front speed=0、ego speed=0、sideあり、Start直後。
  - 期待: 静止距離だけではSafetyBrakeへ入らない。
- 前車発進失敗: 上記からego speedだけ増加、distance減少。
  - 期待: required decelがEmergency域へ入りSafetyBrake。
- gate1相当: 通常走行中の停止対象。
  - 期待: SafetyBrake維持。
- gate2相当: gapがある停止車。
  - 期待: LowSpeedAvoidance維持。
- V2Xなし/単独走行。
  - 期待: start-grid suppressionなし、通常MPCに影響なし。

### Runtime

1. `make autoware-build`
2. 対象package unit test
3. `make dev3`
4. d1〜d3の `/awsim/state`、V2X debug、速度、SafetyBrake遷移を時系列比較
5. 前車を停止させる再現ケースでhard emergencyを確認
6. `make gate1` / `make gate2` の停止・回避退行確認

記録する指標:

- Start受信時刻
- grace arm/expire時刻
- 各車両の初動時刻と1 m/s到達時刻
- front distance / front speed / ego speed / required decel
- `SafetyBrake`開始・解除理由
- solver failure連続数
- collision / wall contact / penalty

## Documentation migration

実装と同じ変更で `docs/spec/mpc-integration.md` を更新する。

- `v2x_start_grid_grace_time` の起点が `/awsim/state == Start` であること。
- 猶予は静止グリッド由来の停止判定だけへ作用すること。
- `EmergencyBrake`、fail-safe、collisionを抑制しないこと。
- 2026公式スタート配置がWIPであり、現在値がローカル暫定であること。

topic/service/message契約は変わらないため、`docs/interface/` のschema変更は予定しない。実装時にinterface guardianで再確認する。

## Rollback

- `v2x_start_grid_grace_time: 0.0` で新しい抑制を無効化できる。
- rollback時も通常 `SafetyBrake` とfront-risk判定は維持される。
- 問題発生時に `v2x_safety_brake_distance` を縮小して代替しない。
- Boost、trajectory、domain速度設定へ問題を転嫁しない。
