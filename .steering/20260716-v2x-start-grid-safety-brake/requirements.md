# V2X Start Grid SafetyBrake Requirements

作成日: 2026-07-16
更新日: 2026-07-16
状態: Implemented / dev3 startup verified（追加safety regression pending）

## 目的

`make dev3` の同時走行スタートで、初期グリッド上の停止車両を通常走行中の停止障害物として扱い、2台目以降が不要な `SafetyBrake` に入る問題を解消する。

`SafetyBrake` 自体は弱めず、スタート配置に対する一時的な抑制と、実際の接近・衝突リスクに対する緊急停止を分離する。

## 背景と観測事実

対象 run:

- `output/20260716-073130/d2/autoware.log`

観測内容:

- d2 は d3 を前方約 3.42 m、速度 0.00 m/s の車両として検出した。
- `v2x_start_grid_grace_time=5.0` の間は `grace=1` で `Follow` を維持した。
- V2X behavior の初回評価から約5秒後に `grace=0` となった。
- 直後に `Follow -> SafetyBrake`、理由 `inside stopping distance` へ遷移した。
- `SafetyBrake` 中は `limit=0.00`、`plan_N=0` となり、MPC solver failure が最大163周期継続した。
- d3 が移動して前方距離が広がるまで d2 は発進できなかった。

現行実装では猶予時間の起点が `/awsim/state == Start` ではなく、`evaluate_v2x_behavior()` の初回呼び出し時刻になっている。そのため、AWSIM の `Grounded` / `Ready` 待機中に猶予時間を消費し、実際のレース開始時には猶予が失効し得る。

## ユーザー決定事項

- スタート直後の初期配置だけを理由に `SafetyBrake` へ入る挙動は修正する。
- `SafetyBrake` の通常走行・障害物停止・衝突回避機能は維持する。
- 距離閾値の単純な縮小や `SafetyBrake` の全面無効化では対応しない。
- 猶予時間の起点は AWSIM のレース開始状態に合わせる。

## 機能要件

### R-START-01: Start epoch

- シミュレーションでは `/awsim/state` の正規化済み `Start` 遷移をスタート猶予の起点とする。
- count開始では物理発進が車両別`Start`より先行するため、`Ready`で静止グリッド抑制だけをprepared状態にする。設定時間の計測epochは引き続き`Start`とする。
- V2X behavior の初回評価、odometry 初回受信、初期姿勢設定、control mode request を猶予時間の起点にしない。
- `Ready`から`Start`までの待機時間でStart後の猶予を消費しない。
- 同一セッション内の重複 `Start` でepochを延長または再armしない。
- `Spawned`、`Grounded`、`Finish`、resetなど明確なセッション境界で前セッションの猶予状態を破棄する。`Ready`は現セッションのprepared状態へ進める。

### R-START-02: Start-grid context

- スタート猶予は `use_sim_time=true` かつレース `Start` を受信したセッションだけで有効にする。
- 猶予は複数車両の初期配置を示す V2X context がある場合にだけ、停止車向け判定へ作用させる。
- 初期静止frontのtarget IDはsession内で1台だけ保持し、moving-front閾値以下の発進立ち上がりで停止判定が再発火しないようにする。target変更、side context離脱、moving-front閾値超過で通常判定へ戻す。
- 単独走行、V2X車両なし、実車起動では通常の `SafetyBrake` 判定を変更しない。
- 猶予時間は既存 `v2x_start_grid_grace_time` を使用し、`0.0` で無効化できることを維持する。

### R-SAFE-01: Suppression scope

- 猶予中に抑制できるのは、初期配置の静止車両に対する次の通常判定に限定する。
  - 停止車向け `inside stopping distance`
  - 停止車を対象とした `LowSpeedAvoidance` の早期開始
- odometry stale、非有限値、solver failure、operator stop、control disable、collision、壁制約など既存fail-safeを抑制しない。
- moving-front用判定、通常走行中の `SafetyBrake`、猶予終了後の判定を変更しない。

### R-SAFE-02: Hard emergency priority

- 猶予中でも、実際の接近を示す `FrontRiskLevel::EmergencyBrake` は常に `SafetyBrake` を優先する。
- 前方距離が減少し、相対接近速度または必要減速度が緊急域へ入った場合は、初期グリッド抑制を解除する。
- 極端な近距離、collision検知、入力異常では安全側へ倒す。
- 前車が発進しない場合でも、後車が無制限に接近を続けないことを受け入れ試験で確認する。

### R-STATE-01: State transition

- `Start` 前に `SafetyBrake` で待機していても、`Start` 後に静止グリッド抑制条件を満たせば、次の正常な制御周期で `Follow` または `Cruise` へ復帰できる。
- 猶予時間終了後は既存の通常V2X FSMへ戻る。
- `SafetyBrake` への遷移は既存どおり即時許可し、緊急停止をstate holdで遅らせない。
- スタート猶予の開始・抑制・終了・緊急解除理由をログで追跡できるようにする。

### R-CLOCK-01: Clock and reset

- epochと経過時間は同一のclockで比較し、ROS timeとsteady clockを混在させない。
- `/clock` の巻き戻り、AWSIM reset、ノード再起動で負の経過時間や猶予の永久継続を発生させない。
- 非有限または不正な時刻入力では抑制を有効にせず、安全側へ倒す。

### R-CONFIG-01: Configuration

既存設定を維持する。

```yaml
mpc:
  v2x_start_grid_grace_time: 5.0
```

- 既定値と有効値は実走確認後に決定する。
- 新しい閾値が必要な場合は `v2x_start_grid_*` 名前空間相当の明示的なconfigとし、通常の `v2x_safety_brake_*` を流用して意味を変えない。
- 負値、NaN、Infは拒否または0へ安全に正規化し、挙動をテストで固定する。

## 非機能要件・制約

- `/awsim/state` のtopic名、型、状態文字列を変更しない。
- `/control/command/control_cmd` のtopic名、型、最終出力責務を変更しない。
- `/v2x/vehicle_positions` と `v2x_msgs` の契約を変更しない。
- Domain 0と車両Domain 1..Nの分離を変更しない。
- `aichallenge_system/`、AWSIM、評価FSMは変更せず、参加者領域へ閉じる。
- Boost、domain別速度・加速度、trajectory、localization設定を今回変更しない。
- `SafetyBrake` の距離閾値を小さくして見かけ上回避する対応は行わない。
- 2026公式のスタート配置・間隔はWIPであり、固定グリッド座標や固定車間をコードへ埋め込まない。

## 受け入れ条件

### Unit / component

- Start前にV2X評価を長時間繰り返しても、Start後の猶予時間が消費済みにならない。
- 正常なStartを1回受信すると、設定時間だけ静止グリッド抑制が有効になる。
- 重複Startで猶予時間が延長されない。
- Finish/reset後は前セッションの状態を引き継がない。
- 猶予中でも `EmergencyBrake` 条件では `SafetyBrake` になる。
- 前車が静止したまま後車が接近したケースで、衝突前に `SafetyBrake` へ戻る。
- 猶予終了後は従来の停止距離判定が働く。
- `v2x_start_grid_grace_time=0.0` では従来の通常判定になる。

### Runtime

- `make dev3` でd1〜d3が初期配置だけを理由にStart直後の停止連鎖へ入らない。
- d2ログでStart後の猶予開始が確認でき、Start前のV2X評価時間を引き継がない。
- d2がd3の初期配置を理由に `SafetyBrake` へ固定されず、正常に発進する。
- 前車を意図的に停止させる再現ケースでは、接近時に `SafetyBrake` が再び働く。
- 車両接触、壁接触、NaN / Inf、control command spikeを増やさない。
- `make gate1` / `make gate2` 相当の停止・低速回避を退行させない。
- `make autoware-build` と対象package testが成功する。

## 対象外

- Boost有効台数やBoost発動戦略の変更。
- `a_max`、`v_max`、domain別スタート速度上限の調整。
- trajectory CSVやスタート位置の変更。
- 通常走行中の追い越し戦略全体の再設計。
- AWSIMまたは評価基盤のスタートFSM変更。
- 実車での発進制御変更。

## 未確定事項

- 2026公式環境におけるスタート位置、最小車間、横並び/縦並び構成。
- 猶予時間の最終値。現行5秒は2025由来のローカル暫定値として扱う。
- hard emergency用に専用最小距離が必要か、既存front-risk required decelだけで十分か。
- Start直後に前車が発進失敗したケースの公式評価上の期待挙動。
