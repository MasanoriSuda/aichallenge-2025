# V2X Overtake Stability Design

作成日: 2026-07-11  
更新日: 2026-07-11  
状態: Implemented / Runtime Verification Pending

## 方針

現行の `V2XGapPlanner`、`V2XBehaviorState`、`OvertakeLinePhase`、front risk arbitrationを維持し、
次の責務を分離する。

1. 他車観測の安定化。
2. 左右gap候補とcurve実行許可の評価。
3. OvertakeLineの状態遷移。
4. desired velocityと安全上限の調停。
5. MPC問題投入前のfeasibility検査。

単に曲率閾値やgap幅を緩和するだけでは、最初のヘアピン接触とOSQP失敗を再発させる。
先に状態遷移と観測性を直し、その後に実測データで閾値を調整する。

## 現行フローと問題点

```text
/v2x/vehicle_positions
        |
        v
V2XGapPlanner tracker
        |
        v
evaluate_v2x_behavior
  |- curve forbidden ----------------------> Follow（gap未評価、plan_N=0）
  |- front/side瞬時分類
  |- pass sideを片側だけ選択
  `- gap guard
        |
        v
V2XBehaviorState::Overtake
        |
        v
OvertakeLine Idle -> ShiftOut -> Pass -> Return / Recovery
        |
        v
xr/lb/ub/velocity limitをMPCへ反映
        |
        v
OSQP --failure--> deceleration fallback
```

主要な構造問題:

- curve不許可時は「gapがない」のではなく「gapを調べていない」が、状態上区別されにくい。
- front/sideの1周期欠落が対象clearとして扱われ、PassがReturnへ落ちる。
- Returnへ落ちた後、25 ms後にOvertakeを再取得してもShiftOut/Passへ復帰しない。
- planner共通の1.8 mとOvertake guardの0.8 mが別段階で作用し、実効必要幅が分かりにくい。
- Overtakeは速度上限を外すだけで、速度参照を維持しない。
- RecoveryはFollow用3.0 m/sを暗黙流用する。
- phase遷移時の横目標と制約がOSQP infeasibleを起こし得る。

## 変更後の論理構成

```text
V2X message
   |
   v
OpponentObservationStabilizer
   |- tracked target ID
   |- front / side / rear hysteresis
   `- stale / jump verdict
   |
   v
OvertakeCandidateEvaluator
   |- left candidate
   |- right candidate
   |- raw/post-margin width
   |- curve entry / continue permission
   `- reachable metrics + structured reason
   |
   v
OvertakePhaseMachine
   |- target lock
   |- Idle / ShiftOut / Pass / Return / Recovery
   `- reacquire / clear confirmation
   |
   +-----------------------+
   |                       |
   v                       v
OvertakeVelocityArbitrator OvertakeLine target generator
   |                       |
   +-----------+-----------+
               v
       MpcInputPreflight
               |
               v
             OSQP
```

初期実装ではROS nodeを分割しない。`mpc_controller_cpp.cpp`内のロジックをtest可能なpure C++ coreへ
段階的に切り出し、nodeはV2X message、現在状態、configのadapterとして残す。

候補ファイル:

```text
include/multi_purpose_mpc_ros/v2x_overtake_core.hpp
src/v2x_overtake_core.cpp
test/test_v2x_overtake_core.cpp
```

最終ファイル名は既存package命名に合わせて変更してよいが、判定coreがrclcpp、OSQP、GUIへ依存しない
境界を維持する。

## データ構造

### StableOpponent

```cpp
struct StableOpponent
{
  std::string vehicle_id;
  double longitudinal_m;
  double lateral_m;
  double speed_mps;
  double last_seen_sec;
  OpponentRegion region;  // Front, Side, Rear, Unknown
  bool stale;
};
```

`vehicle_id`が空または不安定な場合に備え、位置連続性fallbackを使うかは実装前に公式/実環境を確認する。
異なるIDを距離だけで無条件に同一対象へ結合しない。

### GapCandidateResult

```cpp
enum class GapRejectCode
{
  None,
  NoObstacleInHorizon,
  RawWidth,
  PostMarginWidth,
  ConsecutivePoints,
  PrepareDistance,
  GapTime,
  LateralAcceleration,
  InnerCurve,
  HardForbidden,
  MultiFrontPolicy,
  VehicleVehiclePolicy,
  InvalidBounds,
};

struct GapCandidateResult
{
  int pass_side_sign;
  bool geometrically_feasible;
  bool execution_allowed;
  double raw_min_width_m;
  double post_margin_min_width_m;
  double first_gap_distance_m;
  double first_gap_time_sec;
  double required_lateral_accel_mps2;
  GapRejectCode reject_code;
};
```

`geometrically_feasible`と`execution_allowed`を分けることで、curve中に通路が見えても実行禁止であることを
ログとテストで説明できる。

### VelocityDecision

```cpp
struct VelocityDecision
{
  double desired_velocity_mps;
  double hard_upper_limit_mps;
  double applied_reference_mps;
  VelocityLimitSource limiting_source;
};
```

`desired_velocity`はMPC objectiveの参照でありhard minimumではない。`hard_upper_limit`はdomain、curve、
front risk、SafetyBrakeなどの最小値とする。

## 他車観測とhysteresis

### 対象lock

- Overtake開始時に対象vehicle IDとpass sideをlockする。
- 対象の最新longitudinal/lateral/speedを毎周期更新する。
- 1周期見えないだけではclearにしない。
- V2X timeoutまたはposition jumpが継続した場合は`TargetStale`としてRecoveryへ倒す。

### region遷移

instantaneousなfront/side判定は候補値として残すが、phase遷移にはstable regionを使う。

```text
Front -- lateral shift --> Side -- rear clearance confirmed --> Rear/Clear
   \---- short missing sample ---- hold previous region
```

暫定config候補:

```yaml
v2x_overtake_target_hold_sec: 0.30
v2x_overtake_clear_confirm_sec: 0.30
v2x_overtake_target_stale_sec: 1.00
```

値は40 Hzを前提としたローカル初期値であり、公式値ではない。

### pass完了

`!has_front_vehicle && !has_side_vehicle`だけでReturnを開始しない。lock対象について、少なくとも次を確認する。

- signed longitudinalが後方側へ移った。
- rear clearanceが`return_clear_distance`以上。
- clear状態が`clear_confirm_sec`継続した。
- 復帰側に別のside vehicleがいない。

## 左右gap評価

### 評価順序

1. base `lb/ub` を検証。
2. 他車占有intervalを車両半径、prediction margin、covarianceで膨張。
3. left/rightのraw free intervalを両方生成。
4. vehicle-vehicle / multi-front policyを適用。
5. wall clearance marginを適用。
6. 連続点、prepare distance、gap time、required lateral accelerationを計算。
7. curve entry / continue permissionを適用。
8. 実行可能候補からscoreで選ぶ。

### widthの定義

- `raw_width`: 膨張済み車両占有intervalとbase wallの間。
- `post_margin_width`: wall clearanceを引いた後のcenter corridor幅。
- `target_clearance`: 選択targetから車両側/壁側までの余裕。

既存 `gap_min_width=1.8` と `overtake_guard_min_gap_width=0.8` をそのまま二重適用する場合も、
どちらがどのwidthへ作用するかを設定コメントとログへ出す。初期実装では安全値を自動で変更せず、
定義と拒否理由を直した後、dev2/dev3計測で調整する。

### pass side選択

- Idle/Followでは左右を評価し、feasible、連続幅、必要横加速度、現在位置からの距離でscoreする。
- 第一候補がcurve inner側または先で閉じる場合、開始前だけ反対側を再評価する。
- ShiftOut以降はsideをlockし、単一周期のscore差で反転しない。
- lock側が危険になった場合は反対側へ即flipせずRecoveryへ倒す。

## Curve entry / continue

curve判定を次の3段階に分ける。

1. `HardForbidden`: 明示WP範囲、hairpin、境界余裕不足。新規/継続とも禁止。
2. `SoftEntryForbidden`: 先読み曲率超過。原則新規開始禁止。
3. `ContinueAllowed`: すでにSide/Passで、外側、clearance、front risk、横加速度を満たす場合だけ継続候補。

既存の`v2x_overtake_max_curvature=0.03`をいきなり引き上げない。最初に実コース上で
`max_abs_kappa`、禁止率、壁clearanceを可視化し、entry/continueを別々に調整する。

## OvertakeLine状態遷移

```text
Idle
  |
  | candidate selected
  v
ShiftOut ------ target/gap lost ------> Recovery
  |                                      |
  | lateral reached                     | safe center reached
  v                                      v
Pass -------- target safely passed ---> Return ---> Idle
  |             ^                         |
  | short loss  | reacquire same target   |
  +-------------+-------------------------+
```

### 再取得規則

- Pass中の短時間欠落はPassを保持する。
- PassからReturnへ入った直後に同一対象を再取得し、まだ後方clear未成立なら、gap/curve/riskを再確認する。
- 再確認が成立しReturn開始から短時間ならPassへ戻せる。
- Returnが十分進んだ後、またはsideが変わる場合は新規ShiftOutへ即遷移せずRecovery/Idleを経由する。
- Recoveryは安全目的のため、単一周期の再取得だけで解除しない。

暫定config候補:

```yaml
v2x_overtake_reacquire_enabled: true
v2x_overtake_reacquire_window_sec: 0.50
v2x_overtake_reacquire_max_return_progress: 0.25
```

## 速度調停

### 上限と参照を分離

現行は主に上限だけを調整する。変更後は次を分ける。

```text
cruise desired velocity
       |
       +-- Overtake entry speed
       +-- front speed + desired advantage
       v
Overtake desired velocity（soft reference）
       |
       v
min(global hard cap,
    active domain cap,
    ref_vel cap,
    curvature cap,
    front-risk cap,
    phase safety cap)
       |
       v
applied MPC reference
```

候補式:

```text
overtake_desired = max(
  cruise_desired,
  entry_speed,
  front_speed + configured_advantage)

applied_reference = min(overtake_desired, hard_upper_limit)
```

これはsoft referenceであり、`umin`へ速度下限を設定しない。横制約やSafetyBrakeと競合した場合は安全上限を優先する。

暫定config候補:

```yaml
v2x_overtake_speed_reference_enabled: false
v2x_overtake_speed_advantage: 1.0       # [m/s]
v2x_overtake_hold_entry_speed: true
v2x_overtake_recovery_velocity: 3.0     # [m/s], follow_velocityから分離
v2x_overtake_speed_debug_log_enabled: true
```

初期rolloutでは`speed_reference_enabled=false`でビルド・unit testを通し、controlled dev2で有効化する。

### domain start maximum

config読込時にglobal hard maximumを失わない構造へ変更する。

現行の`domain_start_v_max_duration`は、最初のodometryでMPCを初期化した時刻`t_start_`をepochにする。
これはAWSIMの`Grounded/Ready`待機を含むため、最新D2では実際の`Start`より約16秒前から計時され、
15秒設定が走行開始前に失効した。速度値の`min()`問題と、epoch問題を別々に修正する。

```text
global_hard_max = mpc.v_max
normal_domain_max = domain_v_max[domain] or global_hard_max
active_domain_max = start期間中かつ定義あり
  ? domain_start_v_max[domain]
  : normal_domain_max
effective_domain_max = min(global_hard_max, active_domain_max)
```

この結果、D2の37 km/hはglobal 40 km/h以下なので15秒間有効となり、終了後20 km/hへ戻る。
切替で加速度stepを出さず、実際のcommandは既存`a_min/a_max`で制限する。

start epochは公式`/awsim/state == Start`のsession内立上がりを使う。既存Start Boost guardが保持する
race-session状態と責務を共有できるか確認し、同じstateを別々の文字列判定で解釈しない。reset後は
新sessionとしてepochを再設定し、Ready中の待機やnode初期化だけではtimerを開始しない。

### CSV速度との境界

本作業ではtrajectory CSVの`vx_mps/ax_mps2`をruntime参照へ接続しない。Editorのoffline speed metadataと
Overtake runtime velocity policyを混同せず、READMEの既限制約を維持する。

## MPC preflightとsolver失敗

### preflight

Overtake targetを反映した問題についてOSQP前に次を検査する。

- 全`lb/ub/target/reference`がfinite。
- 全horizonで`lb <= ub`。
- targetがwall margin適用後interval内。
- phase最初のtarget変化が`max_target_change`内。
- required lateral accelerationが上限内。
- steering rate制約と前回control horizonの初期値が矛盾しない。

preflight不成立ならOvertakeを開始せず、構造化reasonを付けてFollowを維持する。

### solver failure

- fail-safe減速は維持する。
- solver失敗を毎周期counterへ記録し、開始・継続・復帰をthrottleとは別に状態として持つ。
- Overtake開始後に連続失敗した場合は同じShiftOut targetを再投入せずRecoveryへ倒す。
- 連続回数しきい値は制御周期と安全性を考慮してconfig化するが、初期値はテスト後に確定する。

## Config変更案

新規keyは未指定時に現行互換となる既定値を持つ。

| key候補 | 初期値候補 | 役割 |
|---|---:|---|
| `v2x_overtake_target_hold_sec` | `0.30` | 短時間の対象欠落を保持 |
| `v2x_overtake_clear_confirm_sec` | `0.30` | Return前のclear確認 |
| `v2x_overtake_reacquire_enabled` | `false` | Return直後の同一対象再取得 |
| `v2x_overtake_reacquire_window_sec` | `0.50` | 再取得可能時間 |
| `v2x_overtake_try_both_sides` | `false` | 開始前の左右候補評価 |
| `v2x_overtake_speed_reference_enabled` | `false` | Overtake soft速度参照 |
| `v2x_overtake_speed_advantage` | `1.0` | 前車に対する速度差 |
| `v2x_overtake_hold_entry_speed` | `true` | 安全上限内で進入時参照を保持 |
| `v2x_overtake_recovery_velocity` | `3.0` | Recovery専用速度上限 |
| `v2x_overtake_solver_failure_limit` | `TBD` | phase中断までの連続失敗数 |

実装時にkey数を減らしてもよいが、検出保持、左右評価、速度参照、Recovery速度、solver失敗の責務を
1つの曖昧なflagへ混在させない。

## テスト設計

### Pure C++ unit tests

synthetic path、bounds、opponent sequenceを使う。

| Case | 期待結果 |
|---|---|
| straight / front1台 / leftのみ | left candidate、Overtake可能 |
| straight / front1台 / rightのみ | right candidate、Overtake可能 |
| 第一候補閉塞 / 反対側open | ShiftOut前だけ反対側を採用 |
| gap幅がrawで不足 | `RawWidth`拒否 |
| wall margin後に不足 | `PostMarginWidth`拒否 |
| curve entry | candidateは記録、execution禁止 |
| Pass中soft curve | continue条件で決定 |
| front 1〜数周期欠落 | Pass保持 |
| target後方clear継続 | Returnへ遷移 |
| Return直後に同一target再取得 | 設定に応じPass復帰またはReturn保持 |
| EmergencyBrake | phaseに関係なくSafetyBrake |
| Recovery | 専用速度上限を使用 |
| Overtake speed | hard capを越えずsoft参照を維持 |
| D2 start override | 期間中37、終了後20、global40以下 |
| Ready待機が15秒超 | Start後から15秒間だけoverride有効 |
| reset後の再走行 | 新sessionのStartでepoch再設定 |

### Integration tests

- config省略・disabled互換。
- YAML parseの負値、NaN相当、不正enum拒否。
- node起動時のdomain速度適用ログ。
- existing path/Boost/trajectory testsの回帰。

### Runtime evidence

最低限記録するtopic/情報:

- `/control/command/control_cmd`
- `/localization/kinematic_state`
- `/v2x/vehicle_positions`
- MPCのV2X debug / OvertakeLine / solverログ
- AWSIM state

現行最新走行はrosbag無効だったため、実装検証では記録を有効にして速度低下のcommand chainを確認する。

## 実装順

1. 現行ログをbaselineとして固定し、synthetic再現testを作る。
2. 判定coreをtest可能な単位へ抽出する。
3. target hysteresisとPass/Return再取得を修正する。
4. 左右候補とgap理由を分離する。
5. curve candidate / execution判定を分離する。
6. Recovery速度をFollowから分離する。
7. Overtake soft速度参照とdomain start maximumを実装する。
8. MPC preflightとsolver失敗時phase中断を追加する。
9. dev2、dev3、gate2、最初のヘアピンを順に検証する。
10. 安全証跡を確認後に新機能flagの既定値を判断する。

## Rollback

- 新規のreacquire、both-side、speed-reference機能は個別flagで無効化できるようにする。
- Recovery専用速度を無効化した場合は現行`follow_velocity`へ戻せる移行期間を設ける。
- domain start maximum修正は起動ログでeffective値を確認できるようにし、問題時は該当mappingを削除できる。
- topic、message、launch entryは変更しないため、rollback時に評価基盤変更を必要としない。

## リスク

- target holdが長すぎると、消えた車両を避け続けて壁側へ残る。
- ReturnからPassへ戻しすぎると、左右振動が再発する。
- both-side探索が毎周期sideを変えるとMPC targetが不安定になる。
- 速度参照を強くしすぎると横制約を満たせずOSQP失敗を増やす。
- curve継続を許しすぎるとヘアピン内側接触を再発させる。
- domain start maximum修正によりD2の実速度が上がるため、スタート直後の多車両接触リスクが増える。
- V2X IDが安定しない場合、target lockの前提が崩れる。

## Documentation

実装完了時に更新する。

- `docs/spec/mpc-integration.md`: candidate/execution、state hysteresis、速度調停、config。
- package `README.md`: 調整手順、debugの読み方、CSV速度がruntimeへ未接続である制限。
- 本steering `tasklist.md`: build/test/dev2/dev3のコマンドと結果。
- 公式V2X仕様が確定した場合は先に`docs/interface/participant-interface.md`を更新する。
