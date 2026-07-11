# AWSIM 2026 Start Dash Boost Design

作成日: 2026-07-11
更新日: 2026-07-11
状態: Implemented / Runtime Verified

## 方針

2025由来の「制御コマンドを高頻度再送するBoost」と、2026公式のAWSIM item commandを完全に分離する。

初回スライスはMPCノードへ小さな状態機械を追加し、`Start` 後の安全な最初の制御機会に1回だけ公式Boost pulseを送る。戦略を後から差し替えられるよう、ROS publish処理と発動判定を分離する。

## Interface Compatibility Check

### Verdict

- Needs migration
- 公式 topic は参加者 Domain N 内で完結するため、新しいDomain bridgeや`aichallenge_system`変更は不要。
- 現行ローカル契約文書には `/awsim/status` と `/awsim/cmd` の正式2026仕様が不足しているため、コード実装前に更新する。

### 維持する契約

- `/control/command/control_cmd`: `autoware_auto_control_msgs/msg/AckermannControlCommand`
- `/awsim/state`: `std_msgs/msg/String`
- Domain 0: AWSIM管理面、Domain 1〜4: 車両面
- `aichallenge_submit.launch.xml` と `control_method=mpc`
- result JSON と `output/latest/`

### 追加する車両面契約

| 方向 | Topic | Type | 用途 |
|---|---|---|---|
| AWSIM → MPC | `/awsim/status` | `std_msgs/msg/Float32MultiArray` | 残数・Boost中状態 |
| AWSIM → MPC | `/awsim/state` | `std_msgs/msg/String` | セッションとStart検知 |
| MPC → AWSIM | `/awsim/cmd` | `std_msgs/msg/Float32MultiArray` | Boost rising-edge command |

## コンポーネント

### `AwsimBoostStartDashGuard`

ROSに依存しない小さな状態判定クラスとして切り出す。

責務:

- `/awsim/state` の正規化とセッション境界管理。
- `/awsim/status` 7要素の検証と必要フィールドの保持。
- status受信時刻のsteady clock管理。
- 制御可否、fail-safe、残数、Boost中、送信済みlatchの判定。
- 1回だけ `Pulse` actionを返す。
- 送信前残数から発動確認を判定する。
- 確認timeout時に再送せず、診断結果だけ返す。

候補API:

```cpp
struct AwsimBoostStatus
{
  double remaining;
  bool is_boosting;
  std::chrono::steady_clock::time_point received_at;
};

enum class BoostAction
{
  None,
  PublishPulse,
};

class AwsimBoostStartDashGuard
{
public:
  void on_awsim_state(std::string_view state);
  bool on_awsim_status(
    const std::vector<float> & data,
    std::chrono::steady_clock::time_point now);
  BoostAction evaluate(
    bool control_enabled,
    bool failsafe_active,
    std::chrono::steady_clock::time_point now);
};
```

実際の公開APIは既存コード規約へ合わせるが、pure testからROS executorなしで判定できる構造を維持する。

### `MPCControllerNode` integration

MPCノード側の責務:

- `use_sim_time=true` かつ `awsim_boost.enabled=true` のときだけBoost I/Oを有効化する。
- `/awsim/state` と既存 `/awsim/status` callbackからguardへ入力する。
- 通常control callbackのfail-safe判定後、正常制御のときだけguardを評価する。
- `PublishPulse` を受けたら `/awsim/cmd` に high/lowを各1回publishする。
- guardが返す状態遷移をROS loggerへ出す。

既存の `/awsim/status` callbackはlap情報に加え、同じ受信データをguardへ渡す。subscriptionを重複作成しない。

## 状態機械

```text
Disabled
  └─ enabled + simulation ─> Armed

Armed
  ├─ status invalid/stale/remaining=0 ─> Armed（送信なし）
  ├─ isBoosting=true ─> Armed（送信なし）
  └─ Start + control enabled + healthy + valid status ─> PulseSent

PulseSent
  ├─ isBoosting=true または remainingが1減少 ─> Confirmed
  ├─ confirmation timeout ─> UnconfirmedSpent
  └─ その他 ─> PulseSent

Confirmed / UnconfirmedSpent
  ├─ duplicate Start/status/control tick ─> 同状態（送信なし）
  └─ Finish後の新しい Spawned ─> Armed
```

`UnconfirmedSpent` も使用済みとして扱う。通信確認が取れない場合の再送は、実際には最初の指令が届いていたときに2回目を消費するため行わない。

## 発動シーケンス

```text
AWSIM                 MPC node                Boost guard
  | /awsim/state=Start   |                         |
  |--------------------->| on_awsim_state(Start)   |
  | /awsim/status [..,remaining,isBoosting]        |
  |--------------------->| on_awsim_status(...)    |
  |                      | control callback healthy|
  |                      |------------------------>|
  |                      |       PublishPulse      |
  |                      |<------------------------|
  | /awsim/cmd [1.0]     |                         |
  |<---------------------|                         |
  | /awsim/cmd [0.0]     |                         |
  |<---------------------|                         |
  | /awsim/status changed|                         |
  |--------------------->| confirm; no retry       |
```

## Pulse publish設計

- message 1: `data = {1.0F}`
- message 2: `data = {0.0F}`
- 同じcallback内でこの順序にpublishする。これは公式teleop実装と同じ。
- QoSは `rclcpp::QoS(rclcpp::KeepLast(10)).reliable()` を候補とする。
- periodic timerでhighを保持しない。
- 次のcontrol callbackで再送しない。
- message配列へ独自フィールドを追加しない。

## Guard条件

| 条件 | 必須値 | 満たさない場合 |
|---|---|---|
| feature | enabled | Disabled |
| environment | `use_sim_time=true` | publishしない |
| session | `Start`受信済み | wait |
| control | enabled | wait |
| safety | fail-safeでない | wait、同セッション再armなし |
| status shape | size >= 7 | reject status |
| status numeric | index 5/6 finite | reject status |
| status age | `<= status_timeout_sec` | wait |
| remaining | `>= 1.0` | skip |
| boosting | `< 0.5` | wait |
| latch | not triggered | pulse |

## セッション管理

- node生成時に `Armed` 相当で開始するが、`Start` がなければ発動しない。
- 大文字小文字を正規化して `spawned`, `start`, `finish` を扱う。
- `Start` 重複では状態を戻さない。
- 発動後、`Ready` やcontrol disableではlatchを解除しない。
- `Finish` を経た後の `Spawned`、または明確な新規 `Spawned` epochだけ次セッションとして扱う。
- 遅れて届いたstateで誤rearmしないため、`Spawned` rearm条件はunit testで固定する。

## Config設計

```yaml
awsim_boost:
  enabled: true
  mode: start_once
  status_timeout_sec: 0.5
  confirmation_timeout_sec: 2.0
```

config parserへ `AwsimBoostConfig` を追加する。

検証:

- `mode in {disabled, start_once}`
- timeoutはfiniteかつ正。
- `enabled=false` または `mode=disabled` はpublisherを動作させない。
- 将来のstrategy追加時も `/awsim/cmd` publisher自体は共通化する。

## Legacy Boostとの境界

現行コードには次の2025由来機構がある。

- `use_boost_acceleration`
- `AckermannControlBoostCommand`
- `/boost_commander/command`
- `boost_commander`による `/control/command/control_cmd` の高頻度再送

今回の実装ではこれらを有効化せず、2026公式Boostの判断・送信に流用しない。全面削除は別作業とし、少なくとも命名とログで混同しないようにする。

## Fail-safeと優先順位

```text
stop request / stale odometry / non-finite / solver failure
  > Boost start request
```

- fail-safeを検出したcontrol cycleでは発動しない。
- fail-safe解除後、まだ未送信で同一セッションのStart中なら発動可能とする。
- ただしstatusがstaleまたは既にBoost中なら送信しない。
- pulse送信後にfail-safeへ入っても再送しない。

## 予定変更ファイル

| ファイル | 変更内容 |
|---|---|
| `multi_purpose_mpc_ros/include/.../awsim_boost_start_dash.hpp` | pure guardと状態定義 |
| `multi_purpose_mpc_ros/src/awsim_boost_start_dash.cpp` | guard実装 |
| `multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp` | state/status入力、publisher、pulse、ログ |
| `multi_purpose_mpc_ros/config/config.yaml` | `awsim_boost`設定 |
| `multi_purpose_mpc_ros/CMakeLists.txt` | library / gtest登録 |
| `multi_purpose_mpc_ros/test/test_awsim_boost_start_dash.cpp` | state machine unit test |
| `docs/interface/participant-interface.md` | 2026公式AWSIM topic契約 |
| `docs/spec/mpc-integration.md` | start-once運用とlegacy境界 |
| `docs/spec/open-questions.md` | 解消済みI/Fと未確定の大会運用回数を分離 |

ファイル名とtarget分割は実装時に既存CMake構造へ合わせて調整する。

## Test設計

### Pure unit test

- Start前はpulseなし。
- status未受信、size不足、NaN / Inf、staleではpulseなし。
- remaining 0ではpulseなし。
- isBoosting trueではpulseなし。
- control disabled / fail-safeではpulseなし。
- 正常条件で `PublishPulse` が1回だけ返る。
- callbackを100回繰り返しても2回目が返らない。
- confirmation timeout後も再送しない。
- remaining減少またはisBoostingでConfirmedへ移る。
- duplicate Start / Readyで再armしない。
- Finish後の新しいSpawnedで次セッションへ1回だけrearmする。

### ROS integration test / runtime check

- `/awsim/cmd` の型が `std_msgs/msg/Float32MultiArray`。
- 1レースにhigh/low各1メッセージだけ観測される。
- `/awsim/status[5]` が1減り、index 6がBoost中を示す。
- Boost無効化時は `/awsim/cmd` publishなし。
- Boost有効/無効のA/Bで、開始後10秒の速度または加速度差を記録する。
- `/control/command/control_cmd` のrateと型に退行がない。

## Documentation migration

`interface-guardian`の確認結果として、実装と同じ変更で次を更新する。

1. `participant-interface.md`へ `/awsim/status`、`/awsim/state`、`/awsim/cmd` を追加。
2. Domain NへAWSIMが直接接続する2026公式説明を追記し、Domain 0管理面と混同しないようにする。
3. `mpc-integration.md`へ公式Boost pulse、start-once、no-retryを記録。
4. `open-questions.md`ではtopic/typeを解決済みにし、オンライン運用回数だけをTBDとして残す。

## Rollback

- `awsim_boost.enabled=false` または `mode=disabled` で即時無効化できる。
- 通常MPC control commandはBoost guardと独立しているため、無効化しても走行制御を維持する。
- 問題発生時もlegacy `use_boost_acceleration` を有効化して代替しない。
