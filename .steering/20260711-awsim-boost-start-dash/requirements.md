# AWSIM 2026 Start Dash Boost Requirements

作成日: 2026-07-11
更新日: 2026-07-11
状態: Implemented / Runtime Verified

## 目的

Automotive AI Challenge 2026 の公式 AWSIM Boost インターフェースを MPC 制御経路から利用できるようにする。

初回実装は効果確認を目的とし、レース開始時のスタートダッシュで1回だけ発動する。コース位置、対戦状況、残り周回などを使う戦略判断は後続作業とする。

## 公式仕様の確認結果

2026-07-11 時点で次を公式仕様として扱う。

- AWSIM は各車両 Domain 1〜4 に直接接続する。
- AWSIM 状態は `/awsim/status`、型は `std_msgs/msg/Float32MultiArray`。
  - index 5: `boostRemaining`
  - index 6: `isBoosting`（`1.0` が Boost 中、`0.0` が非 Boost）
- Boost 指令は `/awsim/cmd`、型は `std_msgs/msg/Float32MultiArray`。
  - index 0 の `boostCommand` を `0.0` から `1.0` 以上へ立ち上げると発動する。
  - 次回発動可能な状態へ戻すには、指令を一度 `0.0` に戻す必要がある。
- Boost 効果は加速度 `+0.5 m/s²`、持続時間10秒。
- `--boosts` の既定値は5だが設定可能である。現在のローカル `dev` / `eval` / `parallel` は2回に設定されているため、回数をコードへ固定しない。
- SW部門ルールはWIPであり、オンライン予選の実運用回数は起動設定または運営発表を正とする。

公式参照:

- [2026 インターフェース仕様](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/specifications/interface.html)
- [2026 シミュレーター仕様](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/specifications/simulator.html)
- [2026 SW部門ルール](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/sw-class.html)
- [公式 teleop Boost 実装](https://github.com/AutomotiveAIChallenge/aichallenge-racingkart/blob/dev/aichallenge/workspace/src/aichallenge_tools/teleop_manager/src/teleop_manager_node.cpp)

## ユーザー決定事項

- 初回はスタートダッシュ時に1回だけ使用する。
- 同一レース中は2回目を発動しない。
- Boost 中に指令を連打しない。
- 効果確認後の使用場所・戦略は今回決めない。

## 機能要件

### 1. 正式 AWSIM インターフェース

- MPC ノードはシミュレーション時のみ `/awsim/state` と `/awsim/status` を購読する。
- MPC ノードはシミュレーション時のみ `/awsim/cmd` publisher を作る。
- `/awsim/boost_cmd`、`std_msgs/msg/Bool`、高頻度制御コマンド再送を使用しない。
- 既存の `AckermannControlBoostCommand` / `boost_commander` は正式 Boost 経路として使用しない。

### 2. スタートダッシュ発動条件

次をすべて満たす最初の制御機会だけを発動候補とする。

1. Boost 機能が config で有効。
2. `use_sim_time=true`。
3. `/awsim/state` で `Start` を受信済み。
4. MPC の自動制御が有効。
5. odometry と通常制御が fail-safe 状態ではない。
6. `/awsim/status` が7要素以上あり、所定の freshness 内。
7. `boostRemaining >= 1.0`。
8. `isBoosting < 0.5`。
9. 現セッションで Boost 指令を未送信。

`Start` の重複通知、status の繰り返し、制御周期の繰り返しでは再発動しない。

### 3. 1回限りの指令

- 発動時は `/awsim/cmd` に `{data: [1.0]}`、続けて `{data: [0.0]}` を各1回だけ publish する。
- publisher は2メッセージの順序と配送を維持できる Reliable / KeepLast QoS を使う。
- high/low の1ペアを送信した時点でセッション内の `triggered` latch を立てる。
- AWSIM 状態確認が遅延・欠落しても自動再試行しない。
- `isBoosting >= 0.5` の間は、未発動状態であっても新しい high edge を送らない。

### 4. セッション境界

- Boost 使用回数は1レースセッションにつき最大1回とする。
- ノード起動直後は未使用状態とする。
- `Start` の再受信だけでは再armしない。
- 前セッション終了後に新しい `Spawned` を観測した場合だけ、次セッション用に再armできる。
- `Ready`、status timeout、control disable、solver failureだけでは再armしない。

### 5. 状態確認とログ

- 指令前の `boostRemaining` と `isBoosting` を記録する。
- 指令後に `isBoosting >= 0.5` または `boostRemaining` の1減少を観測したら、発動確認ログを1回出す。
- 確認 timeout 時は warning を出すが、Boost を再送しない。
- 無効化理由、残数なし、status不足・stale、既に使用済み、Boost中を区別できる throttled log または一回限りログを用意する。

### 6. 設定

設定は参加者領域の `config.yaml` に置く。初期候補は次とする。

```yaml
awsim_boost:
  enabled: true
  mode: start_once
  status_timeout_sec: 0.5
  confirmation_timeout_sec: 2.0
```

- `mode` は初回実装では `disabled` / `start_once` のみを受理する。
- 未知の mode、負または非有限の timeout は起動時 error とする。
- 実車起動では設定が有効でも指令を送らず、シミュレーション専用であることをログへ出す。

## 非機能要件・制約

- `/control/command/control_cmd` の名前、型、publisher責務を変更しない。
- Boost は通常の目標加速度を書き換えず、AWSIM item command として独立して扱う。
- Domain 0 の `/admin/awsim/*` に参加者 MPC から触れない。
- `aichallenge_system/`、AWSIM バイナリ、シミュレータの `--boosts` 設定を今回変更しない。
- SingleThreadedExecutor 内で状態を更新し、Boost用の追加threadを作らない。
- status配列の不足、NaN / Inf、stale、残数なしでは発動しない。
- solver failure、stop request、odometry staleなど既存fail-safe中は発動しない。
- ユーザーの現在の trajectory / domain別MPC設定を変更しない。

## 受け入れ条件

- 正式 topic / message 契約が `docs/interface/participant-interface.md` に反映される。
- `Start` 前、Boost中、残数0、status不足・stale、fail-safe中に指令が出ない。
- 正常条件では `/awsim/cmd` に `[1.0]` と `[0.0]` が1回ずつ、この順序で出る。
- 同一セッションで `Start` / status / control callback が繰り返されても追加publishがない。
- 確認 timeout 後も再送しない。
- 新しい `Spawned` を伴う次セッションでは1回だけ再armできる。
- `make autoware-build` と package test が成功する。
- `make dev` で `boostRemaining` が1減少し、`isBoosting` がBoost中を示す。
- 同条件のBoost無効/有効走行を比較し、開始後10秒間の速度または縦加速度ログに効果が確認できる。
- `/control/command/control_cmd`、FSM、Domain、提出物、result JSON、`output/latest/` の既存契約を壊さない。

## 対象外

- 直線、コーナー立ち上がり、追い越しなどを選ぶ最適化戦略。
- 2回目以降のBoost利用。
- 対戦相手、V2X、順位、残り周回を使うBoost判断。
- 実車でのBoost。
- AWSIM本体、Boost効果、使用可能回数の変更。
- legacy `boost_commander` の全面削除。

## 未確定事項

- オンライン予選で実際に付与されるBoost回数。実装は `boostRemaining` を正として吸収する。
- Start直後の最適な発動遅延。初回は遅延0秒相当、すなわち全発動条件を満たした最初の制御機会とする。
- 物理効果の測定方法と許容差。初回検証で `/vehicle/status/velocity_status` と `/awsim/status` を記録して基準を決める。
