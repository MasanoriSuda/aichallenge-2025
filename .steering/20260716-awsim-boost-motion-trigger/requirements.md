# AWSIM Boost Motion Trigger Requirements

作成日: 2026-07-16
更新日: 2026-07-16
状態: Implemented / dev3 verified

## 目的

現在の `start_once` Boostは `/awsim/state=Start` 後の最初の正常制御周期で発動する。しかし、`make dev3` のcount開始では車両の物理発進が `Ready` 中に始まり、車両別の `Start` は発進から数秒後に届く。

Boostの公式topic、1セッション1回、no-retry、安全条件は維持したまま、発動契機だけを「Start受信」から「実際の前進開始」へ変更し、発車直後にBoost効果を得られるようにする。

## Baseline evidence

対象run: `output/20260716-083853`

| Domain | Ready時刻 [s] | 初回 `ego >= 0.1 m/s` [s] | 現行Boost pulse [s] | 発車からの遅れ [s] |
|---|---:|---:|---:|---:|
| d1 | 1784158758.3646 | 1784158759.1450 | 1784158767.0706 | 7.9256 |
| d2 | 1784158758.3648 | 1784158759.1692 | 1784158766.1199 | 6.9508 |
| d3 | 1784158758.3647 | 1784158759.1965 | 1784158765.4235 | 6.2271 |

全車でBoost pulseと `remaining: 2 -> 1` の確認は成功しているため、公式I/Fやpulse方式ではなく発動epochの不一致が対象問題である。

## ユーザー決定事項

- Boostを `/awsim/state=Start` 受信時ではなく、車両が発車した直後に発動する。
- 既存の公式 `/awsim/status`、`/awsim/cmd` と1回限りのpulse方式を維持する。
- trajectory、通常MPC加速度、Boost効果時間・残数は変更しない。

## 機能要件

### R-TRIGGER-01: 発車準備

- `/awsim/state=Ready` を受信したら、未使用セッションのBoost guardを発車監視状態へ進める。
- `Ready` 受信だけではBoost pulseを送らない。
- `Grounded` やノード起動直後の制御出力では発動しない。
- 重複 `Ready` で監視epoch、使用済みlatch、発動回数を更新しない。
- `Start` はrace-session edge、domain start speed window、start-grid graceの契約として従来どおり扱い、Boost都合で早期生成・読み替えしない。

### R-TRIGGER-02: 実発車エッジ

- 発車はodometry由来の符号付き前進速度が設定閾値以上になった最初の正常control cycleで検出する。
- 初期候補値は `motion_speed_threshold_mps: 0.1` とする。
- 負速度、Reverse recovery、非finite速度、stale odometryは発車として扱わない。
- nodeがReady後に起動するなど既に閾値を超えている場合も、許容上限速度以内なら最初の正常評価を発車検出として扱う。
- Ready通知が欠落する起動経路では、`Start` 受信時に車速が許容上限以下の場合だけ発車監視をfallback準備できる。
- 既に通常走行速度へ達した後の遅延 `Start` を発車エッジとして扱わない。

### R-TRIGGER-03: 発動window

- 発車検出時刻はsteady clockで1回だけ記録する。
- statusやcontrol条件が同周期で揃っていれば直ちにpulseを送る。
- 一時的なstatus更新遅延は、発車検出後の短い `motion_trigger_timeout_sec` 内だけ待てる。
- 初期候補値は `motion_trigger_timeout_sec: 0.5` とする。
- timeout超過または `max_trigger_speed_mps` 超過後は、そのセッションのstart Boostを見送って使用済み相当とし、直線・コーナー途中で遅れて発動しない。
- 初期候補値は `max_trigger_speed_mps: 1.0` とする。
- clock rollback、非finite設定、負の経過時間では発動せず安全側へ失効させる。

### R-SAFE-01: 発動時の正常制御条件

次をすべて満たすcontrol cycleだけpulse送信を許可する。

1. `awsim_boost.enabled` と対象Domainのoverrideが有効。
2. `use_sim_time=true`。
3. Readyまたは安全なStart fallbackで発車監視済み。
4. 前進速度がmotion閾値以上かつmax trigger速度以下。
5. 自動制御が有効。
6. 通常control commandのpublishが成功。
7. odometry stale、非finite、forced stop、operator stop、MPC solver fallbackではない。
8. V2X `SafetyBrake` 中ではない。
9. stuck recovery中またはrecoveryによるBoost inhibit中ではない。
10. freshかつ有効な `/awsim/status` があり、残数1以上、Boost中ではない。
11. 現セッションでpulse未送信かつmotion trigger未失効。

### R-PULSE-01: 既存pulse契約維持

- `/awsim/cmd` の `Float32MultiArray [1.0] -> [0.0]` を同じcallbackで各1回送る。
- pulse送信時点でセッション内latchを使用済みにする。
- `isBoosting` または残数減少で確認するが、確認timeout後も再送しない。
- duplicate Ready / Start / status / control callbackで再発動しない。
- `Finish -> Spawned` の明確な新セッションだけ再armする現行契約を維持する。
- Finishを省く手動resetでBoostの使用済みlatchを解除しない。

### R-CONFIG-01: 設定

既存 `mode: start_once` は1セッション1回の戦略名として維持し、発動契機を独立設定にする。

```yaml
awsim_boost:
  enabled: true
  mode: start_once
  trigger: first_forward_motion
  motion_speed_threshold_mps: 0.1
  max_trigger_speed_mps: 1.0
  motion_trigger_timeout_sec: 0.5
  status_timeout_sec: 0.5
  confirmation_timeout_sec: 2.0
```

- `trigger` は `awsim_start` と `first_forward_motion` を受理する。
- `awsim_start` は旧挙動へ戻すための明示的rollback optionとする。
- thresholdとtimeoutはfiniteかつ正、max speedはfinite、`max_trigger_speed_mps >= motion_speed_threshold_mps` を必須とする。
- 2026公式環境での最適値が未確定であるため、0.1 / 1.0 / 0.5はローカル暫定値と明記する。

### R-LOG-01: 診断

- Readyによるmotion watch準備を状態変化時に1回記録する。
- Ready準備と発車検出を同一ログclockで記録し、発車速度とReadyからの経過時間を追跡可能にする。
- pulse時刻、発車検出からpulseまでの遅延、送信前残数を記録する。
- timeout、速度上限、SafetyBrake、solver fallback、status staleなどの見送り理由を区別する。
- 40 Hzで同一理由を連打せず、状態変化またはthrottleで出力する。

## 非機能要件・制約

- `/awsim/state`、`/awsim/status`、`/awsim/cmd` のtopic名・型・配列契約を変更しない。
- `/control/command/control_cmd` のtopic、型、最終制御出力責務を変更しない。
- Domain 0と車両Domain 1..Nの分離を変更しない。
- `aichallenge_system/`、AWSIM、`--boosts`、Boost物理効果を変更しない。
- Start-grid SafetyBrake guardのReady/Start状態管理を壊さない。
- Boostのために制御指令や速度値を偽装しない。
- 実車、legacy `boost_commander`、trajectory CSVを変更しない。

## 受け入れ条件

### Unit / component

- Spawned / Grounded / Ready静止中はpulseなし。
- Ready後、0.1 m/s未満ではpulseなし。
- Ready後の最初の正常な0.1 m/s到達でpulse actionを1回だけ返す。
- Start未受信でもReady済みならmotion triggerが成立する。
- Ready欠落時は低速Start fallbackが成立し、高速の遅延Startは成立しない。
- SafetyBrake、solver fallback、forced stop、Reverse、stale odometry/statusではpulseなし。
- 発車後0.5秒以内に正常化しなければ見送り、その後も再送しない。
- duplicate Ready / Startでpulseやwindowを延長しない。
- Finish -> Spawned後の次セッションでだけ1回再armできる。
- 旧 `trigger: awsim_start` のrollback経路が既存test相当で動く。

### Runtime

- `make dev3` でd1〜d3のBoost pulseが各車の初回 `ego >= 0.1 m/s` から0.25秒以内に出る。
- Ready受信から静止中にはpulseが出ない。
- 車両別Startを待たず、d1〜d3の発動時刻差が物理発進時刻差に追従する。
- 各車で残数が1だけ減り、Boost確認後も追加pulseがない。
- Start-gridでSafetyBrake中の車両にはpulseが出ず、正常復帰・発車後にだけ出る。
- collision、wall contact、control spike、solver failureを増やさない。
- `make autoware-build` と対象package testが成功する。

### 追加回帰（follow-up）

- Boost無効/有効A/Bで発車後10秒間の速度または加速度差を記録する。
- status stale、rollback trigger、単独車両runを追加確認する。

## 対象外

- 2回目以降のBoost戦略。
- コース位置、順位、V2X、追い越しを使うBoost最適化。
- Boost効果量、持続時間、使用可能回数の変更。
- domain別加速度・最高速度、trajectory、start-grid配置の変更。
- AWSIM state machineやcount開始処理の変更。

## 未確定事項

- `motion_speed_threshold_mps` の最終値。0.1 m/sは直近runと既存停止判定を基にした暫定値。
- `max_trigger_speed_mps` とtimeoutの最終値。
- 公式評価環境でReadyが必ず通知されるか。欠落経路は低速Start fallbackで吸収する。
- Boost発動直後の多車両車間と壁接触への影響。
