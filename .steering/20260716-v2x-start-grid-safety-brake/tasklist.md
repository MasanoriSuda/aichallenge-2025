# V2X Start Grid SafetyBrake Tasklist

作成日: 2026-07-16
更新日: 2026-07-16
状態: Implemented / dev3 startup verified（追加safety regression pending）

## Definition of Done

- Start猶予の起点がV2X初回評価ではなく `/awsim/state == Start` になっている。
- Start前の待機時間で猶予を消費しない。
- 初期グリッドの静止車両だけを理由にd2/d3が `SafetyBrake` へ固定されない。
- 猶予中でも実接近・EmergencyBrake・既存fail-safeは抑制されない。
- 前車が発進しない場合に後車が衝突前に停止できる。
- 通常走行のSafetyBrake、gate1停止、gate2低速回避を退行させない。
- `make autoware-build` と対象package testが成功する。
- `make dev3` の3台ログで発進改善と安全性を確認できる。
- `docs/spec/mpc-integration.md` と実装が一致する。
- ROS topic、Domain、評価、提出物契約を変更しない。

## 0. Baseline evidence

- [x] `output/20260716-073130/d1..d3/autoware.log` の発進時系列を比較する。
- [x] d2がfront=d3、distance=3.42 m、front speed=0であることを確認する。
- [x] `grace=1 -> 0` の直後に `Follow -> SafetyBrake` へ入ることを確認する。
- [x] d2で `limit=0.00`、`plan_N=0`、solver failure最大163周期を確認する。
- [x] 現行猶予epochが `first_v2x_behavior_eval_sec` であることを確認する。
- [x] `/awsim/state` callbackに既存のsession trackingがあることを確認する。

## 1. Specification and interface check

- [x] `requirements.md` のhard emergency条件を実装前に確定する。
- [ ] 2026公式スタート配置・車間に更新がないか確認し、WIP事項を分離する。
- [ ] `interface-guardian`でtopic/service/message/Domain契約に変更がないことを確認する。
- [x] `aichallenge_system/`を変更せず参加者領域だけで完結することを確認する。
- [x] `docs/spec/mpc-integration.md` の現行説明と変更後の意味差分を整理する。

## 2. Start-grid guard

- [x] guardをpure classへ分離するか、既存translation unit内に置くか決定する。
- [x] `Disabled / WaitingForStart / Prepared / Grace / Expired` のsession状態を実装する。
- [x] 正規化済み `/awsim/state` の `Start` でepochを1回だけarmする。
- [x] 重複Startでepochを更新しない。
- [x] Finish/Spawned/Grounded/reset等のsession境界でdisarmする。
- [x] epochとelapsedを同一clock domainで扱う。
- [x] clock rollback、NaN、Infで抑制を無効にする。
- [x] `v2x_start_grid_grace_time=0.0` の無効化経路を維持する。

## 3. V2X behavior integration

- [x] `first_v2x_behavior_eval_sec` 起点の猶予計算を置き換える。
- [x] start-grid contextをStart phase、front/side、stationary条件から構成する。
- [x] static stopped-vehicle判定だけへsuppressionを適用する。
- [x] `FrontRiskLevel::EmergencyBrake` からstart-grid suppression条件を外す。
- [x] `inside stopping distance` はEmergencyBrake判定後に評価する。
- [x] LowSpeedAvoidanceの開始/継続とstart-grid suppressionの関係を維持する。
- [x] Start前の既存停止状態からReady後に正常復帰できることを確認する。
- [x] 通常Follow/Overtake/Cruiseのstate holdを退行させない。

## 4. Diagnostics

- [x] grace arm、expire、session clearを状態変化ログへ追加する。
- [x] static stop suppressionの対象ID・距離・速度を状態変化時に追跡可能にする。
- [x] EmergencyBrakeによるsuppression override理由をログへ出す。
- [x] 既存V2X debugの `grace` fieldを維持する。
- [x] 40 Hzで同一ログを連打しないことを確認する。

## 5. Unit tests

- [x] Start前の長時間評価で猶予が消費されないtest。
- [x] Start後に設定時間だけGraceになるtest。
- [x] duplicate Startでepochが延長されないtest。
- [x] session境界後の次Startで再armするtest。
- [x] grace time 0のtest。
- [x] clock rollback / NaN / Infのtest。
- [x] d2相当の静止gridでSafetyBrakeを抑制するtest。
- [x] 猶予中のEmergencyBrakeを抑制しないtest。
- [ ] 前車発進失敗・gap減少でSafetyBrakeへ入るtest。
- [x] side contextなしではstart-grid suppressionを返さないtest。
- [ ] gate2相当のLowSpeedAvoidance継続test。

## 6. Build and static verification

- [x] `make autoware-build`
- [x] `colcon test --packages-select multi_purpose_mpc_ros`
- [x] `colcon test-result --verbose`
- [x] 追加C++ファイルの `clang-format --dry-run --Werror`。
- [x] `/control/command/control_cmd` の型・publisher責務に差分がないことを確認する。
- [x] `/awsim/state`、`/v2x/vehicle_positions` のsubscription契約に差分がないことを確認する。

## 7. dev3 runtime verification

- [x] `make down` 後に `make dev3` をクリーン起動する。
- [x] d1〜d3が同一run directoryへ記録されることを確認する。
- [x] 各Domainで `/awsim/state == Start` の受信時刻を記録する。
- [x] grace arm時刻がStart時刻と一致することを確認する。
- [x] Start前のV2X評価時間をStart後の猶予へ算入していないことを確認する。
- [x] d2が初期配置だけを理由にSafetyBrakeへ固定されないことを確認する。
- [x] d1〜d3の初動、1 m/s、5 m/s到達時刻を比較する。
- [ ] solver failure連続数がbaselineの163周期から減少することを確認する。
- [ ] collision、wall contact、penalty、control spikeが増えないことを確認する。
- [ ] Boost有効/無効差とstart-grid修正効果をログ上で分離する。

## 8. Safety regression

- [ ] 前車を停止させた再現ケースで後車がEmergencyBrakeへ入ることを確認する。
- [ ] 前方距離、相対速度、required decel、停止位置を記録する。
- [ ] `make gate1` で停止判定を確認する。
- [ ] `make gate2` でLowSpeedAvoidanceを確認する。
- [ ] V2X欠損・stale・position jumpで危険なsuppressionにならないことを確認する。
- [ ] 単独 `make dev` に挙動差がないことを確認する。

## 9. Documentation and completion

- [x] `docs/spec/mpc-integration.md` にStart epochとSafetyBrake優先順位を反映する。
- [x] 5秒を2025由来のローカル暫定値として明記する。
- [ ] 必要なら `docs/spec/open-questions.md` に公式スタート配置の確認事項を追加する。
- [x] 実行コマンド、run ID、主要時刻、未検証項目をtasklistへ追記する。
- [x] requirements/design/tasklistの状態と更新日を実装・dev3確認済みに更新する。

## 実行記録

### 2026-07-16 Baseline analysis

- run: `output/20260716-073130`
- d2 SafetyBrake開始: `1784154712.705848325`
- d2 SafetyBrake理由: `inside stopping distance`
- baseline front distance: 約3.42 m
- baseline front speed / ego speed: 0.00 / 0.00 m/s
- baseline solver failure peak: 163 consecutive failures
- 判定: Start epoch不一致と静止距離判定の組み合わせが発進遅延の主要因。

### 2026-07-16 Implementation validation

- `make autoware-build`: 成功。25 packages finished。
- `test_start_grid_grace`: 10 tests passed。最終ビルド後の単独再実行も成功。
- 追加C++ファイルの `clang-format --dry-run --Werror`: 成功。
- package全体: 17 CTest中16件成功。今回追加testは成功。
- 既存失敗: `PathCoreCircular.RemovesOneEndpointFromConfiguredFinalVer3Trajectory`。現在の `final_ver3` trajectoryが終端重複を持つという既存test前提と一致しない。今回のStart-grid変更対象外で、trajectoryは変更していない。
- `colcon test-result --verbose`: 上記既存失敗を報告。別packageの古い `build/joycon_contract_guard/package.xml` 欠損warningもあり。
- 未実施: gate1/gate2、前車発進失敗runtime。追加safety regressionとして残す。

### 2026-07-16 dev3 runtime validation

- final run: `output/20260716-083853`
- d1/d2/d3 Ready: `1784158758.3646` 秒付近。ここでpreparedへ遷移し、Start後の5秒は未消費。
- d2: `SafetyBrake -> Follow` は `1784158758.3692` 秒。初期front=d3、distance=3.42 m、speed=0.00 m/s。
- d2: d3が0.63 m/s、0.98 m/sへ立ち上がる間も同一target IDの静止停止判定を抑制。1.0 m/s超過後は通常判定へ復帰。
- 0.1 m/s到達: d1=`1784158759.1450`、d2=`1784158759.1692`、d3=`1784158759.1965`。3台差は約0.052秒（baselineはd2/d1がd3より約2.0秒遅延）。
- 1 m/s到達: d1=`1784158761.1700`、d2=`1784158761.2192`、d3=`1784158760.2215`。
- 5 m/s到達: d1=`1784158769.3450`、d2=`1784158768.3692`、d3=`1784158768.3715`。
- Start arm: d3=`1784158765.4085`、d2=`1784158766.1083`、d1=`1784158767.0604`。各Domainで受信したStartから約5秒後にExpired。
- d2はReady直後にMPC solverが292連続failureから回復。failure数はReadyまでの待機長に依存するためbaseline 163との単純比較対象にしない。
- grace終了付近では通常の `inside stopping distance` SafetyBrakeが再び動作し、その後 `front risk emergency` も記録。SafetyBrake自体は無効化されていない。
- `collision`、`wall contact`、`penalty` に一致するログはなし。control spikeの定量評価は未実施。
- 検証後に `make down` を実行。走行で更新された生成結果JSONは実装差分から除外。

## Rollback check

- [x] `v2x_start_grid_grace_time=0.0` でsuppressionを無効化できる。
- [ ] rollback後も通常SafetyBrakeとfront-riskが機能する。
- [ ] rollbackにBoost、trajectory、domain速度変更を混ぜない。
